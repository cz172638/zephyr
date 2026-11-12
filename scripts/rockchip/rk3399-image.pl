#!/usr/bin/perl
# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0
#
# RK3399 boot images for Zephyr as BL33, without Rockchip blobs:
#
#   BootROM -> U-Boot TPL (DRAM init) -> U-Boot SPL -> FIT {
#       firmware  atf-1        TF-A BL31 segment holding the entry
#       loadables zephyr       Zephyr (os "u-boot": BL31 starts it as BL33)
#                 atf-2..n     the other BL31 segments (SRAM, PMUSRAM)
#       fdt       fdt-1        SPL records the loadables in it, so SPL knows
#                              the BL33 entry (not CONFIG_TEXT_BASE)
#   }
#
# Outputs, written from offset 0 of the medium:
#   <out>-spi.bin  idbloader-spi.img at 0, FIT at --spi-offset (0xe0000)
#   <out>-sd.bin   idbloader.img at sector 64, FIT at sector 16384;
#                  sectors 0..63 are zero (no partition table)
#
# Example:
#   rk3399-image.pl --bl31 tfa/build/rk3399/release/bl31/bl31.elf \
#       --zephyr build/zephyr/zephyr.elf --uboot uboot-build-rp4 \
#       -o 151-rp4-pcie
#   flashcp -v 151-rp4-pcie-spi.bin /dev/mtd0         (or "sf update")
#   dd if=151-rp4-pcie-sd.bin of=/dev/sdX bs=1M conv=fsync

use strict;
use warnings;
use Getopt::Long;
use File::Basename qw(dirname basename);
use File::Path qw(make_path);

my ($bl31, $zephyr, $ub, $out);
my $mkimage;
my $spi_off = 0xe0000;
my $sd_idb = 64;
my $sd_fit = 16384;
my $fdt_load = 0x07f00000;

GetOptions(
	'bl31=s' => \$bl31,
	'zephyr=s' => \$zephyr,
	'uboot=s' => \$ub,
	'mkimage=s' => \$mkimage,
	'spi-offset=o' => \$spi_off,
	'fdt-load=o' => \$fdt_load,
	'o|output=s' => \$out,
) && $bl31 && $zephyr && $ub && $out
	or die "usage: $0 --bl31 bl31.elf --zephyr zephyr.elf --uboot <u-boot build dir> "
	     . "-o <prefix> [--mkimage path] [--spi-offset 0xe0000] [--fdt-load 0x07f00000]\n";

$mkimage //= "$ub/tools/mkimage";
for my $f ($bl31, $zephyr, "$ub/idbloader.img", "$ub/idbloader-spi.img", "$ub/u-boot.dtb",
	   $mkimage) {
	-e $f or die "missing $f\n";
}

sub slurp {
	my ($f) = @_;
	open(my $h, '<:raw', $f) or die "$f: $!\n";
	local $/;
	my $d = <$h>;
	close $h;
	return $d;
}

sub spew {
	my ($f, $d) = @_;
	open(my $h, '>:raw', $f) or die "$f: $!\n";
	print $h $d;
	close $h;
}

# ELF64 little endian: entry and PT_LOAD segments with file content
sub elf_load {
	my ($f) = @_;
	my $d = slurp($f);
	substr($d, 0, 4) eq "\x7fELF" or die "$f: not ELF\n";
	ord(substr($d, 4, 1)) == 2 && ord(substr($d, 5, 1)) == 1
		or die "$f: not ELF64 little endian\n";
	my $entry = unpack('Q<', substr($d, 0x18, 8));
	my $phoff = unpack('Q<', substr($d, 0x20, 8));
	my ($phentsize, $phnum) = unpack('S<S<', substr($d, 0x36, 4));
	my @segs;
	for my $i (0 .. $phnum - 1) {
		my ($type, $flags, $off, $vaddr, $paddr, $filesz, $memsz) =
			unpack('L<L<Q<Q<Q<Q<Q<', substr($d, $phoff + $i * $phentsize, 56));
		next unless $type == 1 && $filesz;
		push @segs, [$paddr, substr($d, $off, $filesz)];
	}
	@segs = sort { $a->[0] <=> $b->[0] } @segs;
	return ($entry, @segs);
}

my $work = "$out.parts";
make_path($work);
my $wd = basename($work);
my (@images, @loadables, $firmware);

my ($bentry, @bsegs) = elf_load($bl31);
my $n = 0;
for my $s (@bsegs) {
	my ($a, $data) = @$s;
	my $name = 'atf-' . ++$n;
	my $file = sprintf("%s/bl31_0x%08x.bin", $work, $a);
	spew($file, $data);
	my $has = $a <= $bentry && $bentry < $a + length($data);
	push @images, {name => $name, file => $file, type => 'firmware', os => 'arm-trusted-firmware',
		       load => $a, entry => $has ? $bentry : $a};
	$firmware = $name if $has;
}
defined $firmware or die "$bl31: entry 0x" . sprintf('%x', $bentry) . " in no segment\n";

my ($zentry, @zsegs) = elf_load($zephyr);
my $zbase = $zsegs[0][0];
my $zend = 0;
for (@zsegs) { my $e = $_->[0] + length($_->[1]); $zend = $e if $e > $zend; }
my $blob = "\0" x ($zend - $zbase);
substr($blob, $_->[0] - $zbase, length($_->[1])) = $_->[1] for @zsegs;
spew("$work/zephyr.bin", $blob);
push @images, {name => 'zephyr', file => "$work/zephyr.bin", type => 'standalone', os => 'u-boot',
	       load => $zbase, entry => $zentry};
push @loadables, 'zephyr', grep { $_ ne $firmware } map { $_->{name} } grep { $_->{name} =~ /^atf-/ } @images;

my $dtb = "$work/u-boot.dtb";
spew($dtb, slurp("$ub/u-boot.dtb"));
push @images, {name => 'fdt-1', file => $dtb, type => 'flat_dt', load => $fdt_load};

my $its = "$out.its";
open(my $h, '>', $its) or die "$its: $!\n";
print $h "/dts-v1/;\n\n/ {\n\tdescription = \"RK3399: TF-A BL31 + Zephyr BL33\";\n";
print $h "\t#address-cells = <1>;\n\n\timages {\n";
for my $i (@images) {
	print $h "\t\t$i->{name} {\n";
	printf $h "\t\t\tdata = /incbin/(\"%s\");\n", $i->{file};
	print $h "\t\t\ttype = \"$i->{type}\";\n";
	print $h "\t\t\tarch = \"arm64\";\n";
	print $h "\t\t\tos = \"$i->{os}\";\n" if $i->{os};
	print $h "\t\t\tcompression = \"none\";\n";
	printf $h "\t\t\tload = <0x%08x>;\n", $i->{load};
	printf $h "\t\t\tentry = <0x%08x>;\n", $i->{entry} if defined $i->{entry};
	print $h "\t\t};\n";
}
print $h "\t};\n\n\tconfigurations {\n\t\tdefault = \"config-1\";\n\t\tconfig-1 {\n";
print $h "\t\t\tdescription = \"zephyr\";\n";
print $h "\t\t\tfirmware = \"$firmware\";\n";
print $h "\t\t\tloadables = " . join(', ', map { "\"$_\"" } @loadables) . ";\n";
print $h "\t\t\tfdt = \"fdt-1\";\n";
print $h "\t\t};\n\t};\n};\n";
close $h;

# external data, 512 byte aligned, as binman builds it for the SPL
my $itb = "$out.itb";
system($mkimage, '-q', '-E', '-B', '0x200', '-f', $its, $itb) == 0 or die "mkimage failed\n";
my $fit = slurp($itb);

my $idb_spi = slurp("$ub/idbloader-spi.img");
length($idb_spi) <= $spi_off or die "idbloader-spi.img larger than the FIT offset\n";
spew("$out-spi.bin", $idb_spi . ("\xff" x ($spi_off - length($idb_spi))) . $fit);

my $idb = slurp("$ub/idbloader.img");
$sd_idb * 512 + length($idb) <= $sd_fit * 512 or die "idbloader.img overlaps the FIT\n";
my $sd = ("\0" x ($sd_idb * 512)) . $idb;
$sd .= "\0" x ($sd_fit * 512 - length($sd));
spew("$out-sd.bin", $sd . $fit);

for my $i (@images) {
	printf "  %-7s load 0x%08x%s %8d B\n", $i->{name}, $i->{load},
		defined $i->{entry} ? sprintf(' entry 0x%08x', $i->{entry}) : ' ' x 17, -s $i->{file};
}
printf "%s-spi.bin %d B (FIT at 0x%x), %s-sd.bin %d B (idbloader sector %d, FIT sector %d)\n",
	$out, length($idb_spi) + ($spi_off - length($idb_spi)) + length($fit), $spi_off,
	$out, length($sd) + length($fit), $sd_idb, $sd_fit;
