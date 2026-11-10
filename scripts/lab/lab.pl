#!/usr/bin/perl
# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0
#
# Host client for the Zephyr lab agent (subsys/lab): drive boards from a
# Linux box with the same protocol the boards use between themselves.
#
#   LAB_TOKEN=secret lab.pl <board-ip> hello
#   LAB_TOKEN=secret lab.pl <board-ip> exec pcie ls
#   LAB_TOKEN=secret lab.pl <board-ip> time [n]
#   LAB_TOKEN=secret lab.pl <board-ip> put <local file> [<slot>|/SD:/path]
#   LAB_TOKEN=secret lab.pl <board-ip> run <file.llext> [args...]   (put + lab ext)
#   LAB_TOKEN=secret lab.pl <board-ip> mod <NNN-name.llext>...   (put + lab mod load)
#
# Exit status of exec/run is the remote command's (0 = pass). The board
# must accept this host: "lab peer <host ip>" there, or no peer set.
# LAB_PORT overrides the port (7777).

use strict;
use warnings;
use IO::Socket::INET;
use IO::Select;
use Time::HiRes qw(time);
use File::Basename;

my $port  = $ENV{LAB_PORT}  // 7777;
my $token = $ENV{LAB_TOKEN} // die "LAB_TOKEN not set\n";
my $host  = shift // usage();
my $op    = shift // usage();
my $id    = int(rand(0x7fffffff)) | 1;
my $chunk = 1200;

my $sock = IO::Socket::INET->new(Proto => 'udp', PeerAddr => $host, PeerPort => $port)
	or die "socket: $!\n";
my $sel = IO::Select->new($sock);

sub usage
{
	print STDERR "usage: lab.pl <ip> hello|exec <cmd...>|time [n]|put <file> [dst]|run <file.llext> [args]|mod <file.llext>...\n";
	exit 2;
}

sub now_ns { return int(time() * 1e9); }

# one request, retried with the same id; $cb->(op, args, payload) returns
# 1 when done
sub xfer
{
	my ($op, $args, $payload, $timeout, $cb) = @_;
	my $rid = $id++;
	my $msg = "LAB1 $token $op $rid" . (defined $args ? " $args" : "") . "\n" . ($payload // "");

	for my $try (1 .. 3) {
		$sock->send($msg) or die "send: $!\n";
		my $end = time() + $timeout;
		while ((my $left = $end - time()) > 0) {
			last unless $sel->can_read($left);
			my $d;
			$sock->recv($d, 2048) // next;
			my ($hdr, $pl) = split(/\n/, $d, 2);
			my ($magic, $rop, $rrid, $rargs) = split(/ /, $hdr, 4);
			next unless $magic eq 'LAB1' && defined $rrid && $rrid == $rid;
			return 1 if $cb->($rop, $rargs // '', $pl // '');
			$end = time() + $timeout;
		}
	}
	return 0;
}

if ($op eq 'hello') {
	my $t0 = time();
	xfer('HELLO', undef, undef, 1, sub { print "$_[1]\n"; 1 }) or die "no answer\n";
	printf "(%.1f ms)\n", (time() - $t0) * 1e3;
	exit 0;
}

sub remote_exec
{
	my ($cmd, $quiet) = @_;
	my (%out, $rc);
	my $ok = xfer('EXEC', $cmd, undef, $ENV{LAB_EXEC_TIMEOUT} // 30, sub {
		my ($o, $a, $pl) = @_;
		if ($o eq 'OUT') {
			$out{int($a)} = $pl;
			return 0;
		}
		return 0 unless $o eq 'END';
		my ($r, $n) = split(/ /, $a);
		return 0 if grep { !exists $out{$_} } 0 .. $n - 1;   # lost one: retry
		$rc = $r;
		return 1;
	});
	die "exec: no answer\n" unless $ok;
	print map { $out{$_} } sort { $a <=> $b } keys %out unless $quiet;
	return $rc;
}

sub put
{
	my ($file, $dst) = @_;
	open(my $fh, '<:raw', $file) or die "$file: $!\n";
	local $/;
	my $data = <$fh>;
	close $fh;
	my $len = length($data);
	my $t0 = time();
	for (my $off = 0; $off < $len || ($len == 0 && $off == 0); $off += 1024) {
		my $res;
		xfer('PUT', "$dst $off $len", substr($data, $off, 1024), 1.5, sub {
			return 0 unless $_[0] eq 'ACK';
			(undef, $res) = split(/ /, $_[1]);
			1;
		}) or die "put: no answer at $off\n";
		die "put: remote error $res at $off\n" if $res != 0;
		last if $len == 0;
	}
	printf "%s -> %s:%s, %d bytes, %.0f ms\n", $file, $host, $dst, $len, (time() - $t0) * 1e3;
}

if ($op eq 'exec') {
	usage() unless @ARGV;
	my $rc = remote_exec(join(' ', @ARGV));
	print STDERR "[rc $rc]\n" if $rc != 0;
	exit($rc == 0 ? 0 : 1);
} elsif ($op eq 'time') {
	my $n = shift // 16;
	my ($best, $off) = (1e18, 0);
	for (1 .. $n) {
		my $t1 = now_ns();
		xfer('TIME', $t1, undef, 0.3, sub {
			my ($o, $a) = @_;
			my $t4 = now_ns();
			return 0 unless $o eq 'TIME';
			my ($r1, $t2, $t3) = split(/ /, $a);
			return 0 unless $r1 == $t1;
			my $rtt = ($t4 - $t1) - ($t3 - $t2);
			($best, $off) = ($rtt, (($t2 - $t1) + ($t3 - $t4)) / 2) if $rtt < $best;
			1;
		});
	}
	die "time: no answer\n" if $best == 1e18;
	printf "board clock - host clock = %.0f ns (best rtt %.0f ns)\n", $off, $best;
	print "(the board counts from boot, the host from the epoch: only drift is meaningful)\n";
} elsif ($op eq 'put') {
	my $file = shift // usage();
	put($file, shift // basename($file));
} elsif ($op eq 'mod') {
	usage() unless @ARGV;
	my @slots;
	for my $f (@ARGV) {
		my $slot = substr(basename($f), 0, 23);
		put($f, $slot);
		push @slots, $slot;
	}
	my $rc = remote_exec(join(' ', 'lab mod load', @slots));
	exit($rc == 0 ? 0 : 1);
} elsif ($op eq 'run') {
	my $file = shift // usage();
	my $slot = substr(basename($file), 0, 23);
	put($file, $slot);
	my $rc = remote_exec(join(' ', 'lab ext', $slot, @ARGV));
	exit($rc == 0 ? 0 : 1);
} else {
	usage();
}
