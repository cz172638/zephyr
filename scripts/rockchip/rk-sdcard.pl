#!/usr/bin/perl
# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
#
# Micro SD (or eMMC) boot images for a Rockchip SoC with a FIT of our own
# (Zephyr as BL33 behind TF-A BL31) in place of U-Boot's:
#
#   <out>-sd.bin    the card from offset 0: MBR, the U-Boot idbloader.img
#                   (TPL + SPL) at sector 64, the FIT at the sector the SPL
#                   reads (SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR of the build) and,
#                   unless --fat-mb 0, FAT partition 1 from 16 MiB with the
#                   --fat-add files
#   <out>-boot.bin  its start up to the end of the FIT, the boot area alone,
#                   for "fwupdate" (FAT partition untouched; same MBR)
#
#   dd if=<out>-sd.bin of=/dev/sdX bs=1M conv=fsync
#
# Example (ROCK Pi S):
#   rk-sdcard.pl --uboot build/u-boot-rps --fit build/zephyr/zephyr.itb \
#       -o rock_pi_s-probe
#
# needs mkfs.vfat (dosfstools) and mcopy (mtools) for the FAT partition

use strict;
use warnings;
use Getopt::Long;
use File::Basename qw(basename);

my ($ub, $fit_file, $out);
my ($idb_sec, $fit_sec, $fat_start, $fat_mb) = (64, 16384, 32768, 32);
my $label = "ZEPHYR";
my @fat_add;

GetOptions(
	'uboot=s' => \$ub,
	'fit=s' => \$fit_file,
	'fat-mb=i' => \$fat_mb,
	'fat-add=s' => \@fat_add,
	'label=s' => \$label,
	'o=s' => \$out,
) && $ub && $fit_file && $out
	or die "usage: $0 --uboot <u-boot build> --fit <itb> -o <out> "
	. "[--fat-mb 32 (0: none)] [--fat-add file]... [--label ZEPHYR]\n";

sub slurp {
	my ($f) = @_;
	open(my $fh, '<:raw', $f) or die "$f: $!\n";
	local $/;
	my $d = <$fh>;
	close($fh);
	return $d;
}

sub spew {
	my ($f, $d) = @_;
	open(my $fh, '>:raw', $f) or die "$f: $!\n";
	print $fh $d;
	close($fh) or die "$f: $!\n";
}

my $fit = slurp($fit_file);
substr($fit, 0, 4) eq "\xd0\x0d\xfe\xed" or die "$fit_file: not a FIT\n";
my $idb = slurp("$ub/idbloader.img");
my ($sec) = slurp("$ub/.config") =~ /^CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR=(\S+)/m;
$fit_sec = oct($sec) if defined $sec;

$idb_sec * 512 + length($idb) <= $fit_sec * 512 or die "idbloader.img runs into the FIT\n";
my $d = ("\0" x ($idb_sec * 512)) . $idb;
$d .= "\0" x ($fit_sec * 512 - length($d));
$d .= $fit;

if ($fat_mb > 0) {
	my $fat32 = $fat_mb >= 512;
	my $fatimg = "$out-fat.img";

	length($d) <= $fat_start * 512 or die "FIT runs into the FAT partition\n";
	unlink $fatimg;
	system('mkfs.vfat', '-F', $fat32 ? '32' : '16', '-n', $label, '-C', $fatimg,
	       $fat_mb * 1024) == 0 or die "mkfs.vfat failed (dosfstools)\n";
	for my $f (@fat_add) {
		system('mcopy', '-o', '-i', $fatimg, $f, '::/' . basename($f)) == 0
			or die "mcopy $f failed (mtools)\n";
	}
	# MBR: disk signature, one FAT16/32 LBA partition, CHS "beyond 8 GB"
	my $mbr = "\0" x 512;
	substr($mbr, 440, 4) = pack('V', 0x5a455048);
	substr($mbr, 446, 16) = pack('C C3 C C3 V V', 0x00, 0xfe, 0xff, 0xff,
				      $fat32 ? 0x0c : 0x0e, 0xfe, 0xff, 0xff, $fat_start,
				      $fat_mb * 2048);
	substr($mbr, 510, 2) = "\x55\xaa";
	substr($d, 0, 512) = $mbr;
	my $boot = $d;
	$d .= "\0" x ($fat_start * 512 - length($d));
	$d .= slurp($fatimg);
	unlink $fatimg;
	spew("$out-boot.bin", $boot);
} else {
	spew("$out-boot.bin", $d);
}
spew("$out-sd.bin", $d);
printf "%s-sd.bin %d B (idbloader sector %d, FIT sector %d, 0x%x B%s)\n", $out, length($d),
	$idb_sec, $fit_sec, length($fit), $fat_mb ? ", FAT $fat_mb MiB from sector $fat_start" : "";
printf "%s-boot.bin %d B (boot area for fwupdate)\n", $out, -s "$out-boot.bin";
