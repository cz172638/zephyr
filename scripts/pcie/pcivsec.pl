#!/usr/bin/perl
# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0
#
# Look into the vendor-specific parts of PCIe config space (Linux host,
# root, sysfs): list VSEC/DVSEC capabilities, dump, snapshot, diff and
# watch any range of a function's 4 KiB config space, and decode what is
# known (so far guessed: the ASMedia ASM1184e per-port training state).
#
#   pcivsec.pl ls                              functions with VSEC/DVSEC
#   pcivsec.pl dump  <bdf> [from [len]]        hex dump (default: all 4 KiB)
#   pcivsec.pl snap  <bdf> <file>              save config space
#   pcivsec.pl diff  <file|bdf> <file|bdf>     changed dwords
#   pcivsec.pl watch <bdf> [ms] [from [len]]   print dwords as they change
#   pcivsec.pl ports <bdf>                     decoded switch port state
#   pcivsec.pl ports <bdf> watch [ms]          ... as it changes
#
# <bdf> as lspci prints it (01:00.0, or 0000:01:00.0). watch first reads
# the range for a second and leaves out the bits that move on their own.
# Switches with per-port registers in every port's config space (PLX/
# Broadcom PEX87xx) work with dump/diff/watch on the port's own bdf.

use strict;
use warnings;
use Time::HiRes qw(time sleep);

my $SYS = '/sys/bus/pci/devices';

sub usage
{
	print STDERR "usage: pcivsec.pl ls | dump <bdf> [from [len]] | snap <bdf> <file> |\n",
		"       diff <a> <b> | watch <bdf> [ms] [from [len]] | ports <bdf> [watch [ms]]\n";
	exit 2;
}

sub bdf
{
	my $b = shift // usage();
	$b = "0000:$b" if $b =~ /^[0-9a-f]{2}:[0-9a-f]{2}\.[0-7]$/i;
	die "$b: no such function\n" unless -e "$SYS/$b/config";
	return lc $b;
}

# config space as a list of dwords (from a function or a snap file)
sub cfg
{
	my $src = shift;
	my $raw;

	if (-f $src && !-e "$SYS/$src") {
		open(my $fh, '<:raw', $src) or die "$src: $!\n";
		local $/;
		$raw = <$fh>;
	} else {
		my $b = bdf($src);
		open(my $fh, '<:raw', "$SYS/$b/config") or die "$b: $!\n";
		sysread($fh, $raw, 4096);
	}
	die "$src: only " . length($raw) . " bytes (not root?)\n" if length($raw) < 256;
	return [unpack('V*', $raw)];
}

sub rd { my ($c, $o) = @_; return $c->[$o >> 2] // 0; }

sub ids
{
	my $c = shift;
	my $id = rd($c, 0);
	return ($id & 0xffff, $id >> 16);
}

# extended capabilities: [where, id, version, header2]
sub ecaps
{
	my $c = shift;
	my @l;
	my %seen;
	for (my $p = 0x100; $p && $p < 0x1000 && !$seen{$p}++;) {
		my $h = rd($c, $p);
		last if $h == 0 || $h == 0xffffffff;
		push @l, [$p, $h & 0xffff, ($h >> 16) & 0xf, rd($c, $p + 4), rd($c, $p + 8)];
		$p = ($h >> 20) & 0xffc;
	}
	return @l;
}

sub hexdump
{
	my ($c, $from, $len) = @_;
	for (my $o = $from & ~0xf; $o < $from + $len; $o += 16) {
		printf "%03x: %s\n", $o, join(' ', map { sprintf '%08x', rd($c, $o + 4 * $_) } 0 .. 3);
	}
}

my $op = shift // usage();

if ($op eq 'ls') {
	for my $b (sort glob("$SYS/*")) {
		$b =~ s{.*/}{};
		my $c = eval { cfg($b) } or next;
		my ($v, $d) = ids($c);
		for my $e (ecaps($c)) {
			my ($w, $id, $ver, $h2, $h3) = @$e;
			if ($id == 0x000b) {
				printf "%s %04x:%04x  VSEC  @%03x ID=%04x Rev=%d Len=%03x\n", $b, $v, $d,
					$w, $h2 & 0xffff, ($h2 >> 16) & 0xf, $h2 >> 20;
			} elsif ($id == 0x0023) {
				printf "%s %04x:%04x  DVSEC @%03x Vendor=%04x ID=%04x Rev=%d Len=%03x\n",
					$b, $v, $d, $w, $h2 & 0xffff, $h3 & 0xffff, ($h2 >> 16) & 0xf,
					$h2 >> 20;
			}
		}
	}
} elsif ($op eq 'dump') {
	my $c = cfg(bdf(shift));
	my $from = hex(shift // '0');
	my $len = hex(shift // sprintf('%x', 0x1000 - $from));
	hexdump($c, $from, $len);
} elsif ($op eq 'snap') {
	my $b = bdf(shift);
	my $f = shift // usage();
	open(my $in, '<:raw', "$SYS/$b/config") or die "$b: $!\n";
	sysread($in, my $raw, 4096);
	open(my $out, '>:raw', $f) or die "$f: $!\n";
	print $out $raw;
	printf "%s: %d bytes -> %s\n", $b, length($raw), $f;
} elsif ($op eq 'diff') {
	my $a = cfg(shift // usage());
	my $b = cfg(shift // usage());
	for (my $o = 0; $o < 4 * @$a; $o += 4) {
		my ($x, $y) = (rd($a, $o), rd($b, $o));
		printf "%03x: %08x -> %08x  xor %08x\n", $o, $x, $y, $x ^ $y if $x != $y;
	}
} elsif ($op eq 'watch') {
	my $b = bdf(shift);
	my $ms = shift // 10;
	my $from = hex(shift // '0');
	my $len = hex(shift // sprintf('%x', 0x1000 - $from));
	my $c0 = cfg($b);
	my %vol;
	my $end = time() + 1;
	while (time() < $end) {
		my $c = cfg($b);
		for (my $o = $from; $o < $from + $len; $o += 4) {
			$vol{$o} |= rd($c, $o) ^ rd($c0, $o);
		}
	}
	my $nvol = grep { $vol{$_} } keys %vol;
	print "watching $b 0x" . sprintf('%x', $from) . "+0x" . sprintf('%x', $len) .
		", $nvol volatile dwords masked; ctrl-c ends\n";
	my $last = cfg($b);
	my $t0 = time();
	for (;;) {
		sleep($ms / 1000);
		my $c = cfg($b);
		for (my $o = $from; $o < $from + $len; $o += 4) {
			my $d = (rd($c, $o) ^ rd($last, $o)) & ~($vol{$o} // 0);
			printf "%9.3f %03x: %08x -> %08x  xor %08x\n", time() - $t0, $o, rd($last, $o),
				rd($c, $o), $d if $d;
		}
		$last = $c;
	}
} elsif ($op eq 'ports') {
	my $b = shift // usage(); # a function, or a snap file
	my $mode = shift // '';
	my $ms = shift // 50;
	my $c = cfg($b);
	my ($v, $d) = ids($c);
	die sprintf("%s: %04x:%04x, no decoder (only ASMedia 1b21:1184 so far)\n", $b, $v, $d)
		unless $v == 0x1b21 && $d == 0x1184;
	# guessed from config snapshots with links up/down (2026-11-12):
	# block 0x400 + 0x80 * n for ports 1/3/5/7, +0x4c byte 3 training
	# flags (0x21 empty, 0x6d receiver detected but no lock, 0x7f L0,
	# 0x13 after the link went down), +0x50 0x00000b00 / 0x07000700 (L0)
	my @port = (1, 3, 5, 7);
	$b = bdf($b) if $mode eq 'watch';
	my $show = sub {
		my ($c, $prev, $t) = @_;
		for my $n (0 .. 3) {
			my $blk = 0x400 + 0x80 * $n;
			my ($f, $s) = (rd($c, $blk + 0x4c), rd($c, $blk + 0x50));
			next if $prev && $f == rd($prev, $blk + 0x4c) && $s == rd($prev, $blk + 0x50);
			my $fl = $f >> 24;
			my $what = $fl == 0x7f ? 'L0' : $fl == 0x21 ? 'empty' :
				$fl == 0x6d ? 'detected, no lock' : $fl == 0x13 ? 'was up, now down' : '?';
			printf "%sport %d: flags %02x (%08b) b2 %02x  +50 %08x  %s\n",
				defined $t ? sprintf('%9.3f ', $t) : '', $port[$n], $fl, $fl,
				($f >> 16) & 0xff, $s, $what;
		}
	};
	$show->($c, undef, undef);
	exit 0 unless $mode eq 'watch';
	my $t0 = time();
	for (;;) {
		sleep($ms / 1000);
		my $n = cfg($b);
		$show->($n, $c, time() - $t0);
		$c = $n;
	}
} else {
	usage();
}
