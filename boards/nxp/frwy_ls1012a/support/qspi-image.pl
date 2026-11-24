#!/usr/bin/perl
# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
#
# Turn a flash image into the Tcl data file qspi_flash (ls1012a-qspi.tcl)
# takes: the 64 byte chunks that are not all 0xff, as big-endian words.
#
#   qspi-image.pl nor-zephyr-shell.bin [offset] > nor-zephyr-shell.tcl

use strict;
use warnings;

my ($in, $off) = @ARGV;
die "usage: $0 image.bin [offset]\n" unless defined $in;
$off = defined $off ? oct($off) : 0;

open(my $fh, '<:raw', $in) or die "$in: $!\n";
my $raw = do { local $/; <$fh> };
close($fh);

my $len = length($raw);
my @chunks;
for (my $i = 0; $i < $len; $i += 64) {
	my $c = substr($raw, $i, 64);
	next if $c =~ /^\xff*$/;
	$c .= "\xff" x ((4 - length($c) % 4) % 4);
	push @chunks, sprintf("{0x%x {%s}}", $i,
			      join(' ', map { sprintf('0x%08x', $_) } unpack('N*', $c)));
}
printf "set img_off 0x%x\nset img_len %d\nset img_chunks {\n%s\n}\n", $off, $len,
	join("\n", @chunks);
printf STDERR "%s: %d bytes, %d chunks of 64 to program\n", $in, $len, scalar @chunks;
