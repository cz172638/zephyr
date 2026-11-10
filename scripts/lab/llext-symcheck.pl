#!/usr/bin/perl
# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0
#
# Do these LLEXT modules load on this firmware? Every undefined symbol of
# a module must be exported by the firmware (EXPORT_SYMBOL, the syscall
# and libc groups) or be another module's export.
#
#   llext-symcheck.pl [-nm <nm>] zephyr.elf mod1.llext [mod2.llext ...]
#
# Exit 0: all resolve. Exit 1: missing symbols listed; the firmware needs
# the exports (a firmware rebuild), or the module uses something it
# should get another way.

use strict;
use warnings;

my $nm = 'nm';
if (@ARGV && $ARGV[0] eq '-nm') {
	shift;
	$nm = shift;
}
my $elf = shift // die "usage: $0 [-nm nm] zephyr.elf mod.llext...\n";
die "usage: $0 [-nm nm] zephyr.elf mod.llext...\n" unless @ARGV;

my %have;
open(my $fh, '-|', $nm, $elf) or die "$nm $elf: $!\n";
while (<$fh>) {
	$have{$1} = 1 if /\s__llext_sym_(\S+)$/;
}
close $fh;
die "$elf: no exported symbols (CONFIG_LLEXT off?)\n" unless %have;

# exports of the modules themselves (one module may use another)
for my $m (@ARGV) {
	open($fh, '-|', $nm, $m) or die "$nm $m: $!\n";
	while (<$fh>) {
		$have{$1} = 1 if /\s__llext_sym_(\S+)$/;
	}
	close $fh;
}

my $bad = 0;
for my $m (@ARGV) {
	my @miss;
	open($fh, '-|', $nm, '-u', $m) or die "$nm $m: $!\n";
	while (<$fh>) {
		push @miss, $1 if /^\s*U\s+(\S+)$/ && !$have{$1};
	}
	close $fh;
	if (@miss) {
		printf "%s: %d missing: %s\n", $m, scalar @miss, join(' ', sort @miss);
		$bad = 1;
	} else {
		printf "%s: ok\n", $m;
	}
}
exit $bad;
