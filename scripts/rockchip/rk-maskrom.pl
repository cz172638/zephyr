#!/usr/bin/perl
# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
#
# USB maskrom boot images for a Rockchip SoC with a FIT of our own (Zephyr
# as BL33 behind TF-A BL31) in place of U-Boot's:
#
#   <out>-usb471.bin  the U-Boot TPL (DRAM init), unchanged
#   <out>-usb472.bin  the U-Boot SPL of the ROCKCHIP_MASKROM_IMAGE build with
#                     the FIT appended; the SPL finds the FIT through its
#                     binman payload symbols (image_pos and size, the size
#                     patched here) and copies it to SPL_LOAD_FIT_ADDRESS
#   <out>-loader.bin  both in one Rockchip loader (boot_merger "BOOT"
#                     format: one 0x471 and one 0x472 entry, RC4 coded,
#                     CRC32), for "rkdeveloptool db"; no flash loader
#                     entries, so not for "rkdeveloptool ul"
#
#   xrock maskrom <out>-usb471.bin <out>-usb472.bin --rc4 on
#   rkdeveloptool db <out>-loader.bin
#
# Example (ROCK Pi S):
#   rk-maskrom.pl --uboot build/u-boot-rps --fit build/zephyr/zephyr.itb \
#       --chip RK3308 --load 0x02000000 -o rock_pi_s-probe

use strict;
use warnings;
use Getopt::Long;

my ($ub, $fit_file, $out, $chip, $load);
my $nm = $ENV{NM} // "aarch64-linux-gnu-nm";

GetOptions(
	'uboot=s' => \$ub,
	'fit=s' => \$fit_file,
	'chip=s' => \$chip,
	'load=s' => \$load,
	'nm=s' => \$nm,
	'o=s' => \$out,
) && $ub && $fit_file && $chip && $out
	or die "usage: $0 --uboot <maskrom u-boot build> --fit <itb> --chip RK3308 "
	. "[--load <lowest load address of the FIT images>] -o <out>\n";

$chip =~ /^RK(\w{4})$/ or die "--chip: RK followed by four characters, e.g. RK3308\n";
# boot_merger chip code: the four characters after "RK", big endian
my $chip_code = unpack("N", $1);

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

my $u472 = slurp("$ub/u-boot-rockchip-usb472.bin");
my %sym;

for (`$nm $ub/spl/u-boot-spl`) {
	$sym{$2} = hex($1) if /^([0-9a-f]+) \S (_binman_payload_prop_(?:image_pos|size))$/;
}
defined $sym{_binman_payload_prop_image_pos} && defined $sym{_binman_payload_prop_size}
	or die "$ub: no binman payload symbols in spl/u-boot-spl (ROCKCHIP_MASKROM_IMAGE?)\n";
my $pos = unpack("Q<", substr($u472, $sym{_binman_payload_prop_image_pos}, 8));
substr($u472, $pos, 4) eq "\xd0\x0d\xfe\xed" or die "$ub: no FIT at payload 0x$pos\n";

my ($fit_addr) = slurp("$ub/.config") =~ /^CONFIG_SPL_LOAD_FIT_ADDRESS=(\S+)/m;
defined $fit_addr or die "$ub: SPL_LOAD_FIT_ADDRESS unset\n";
# the staged FIT must not reach the images it carries
if (defined $load && oct($fit_addr) < oct($load) && oct($fit_addr) + length($fit) > oct($load)) {
	die sprintf("FIT staged at %s (0x%x bytes) overlaps the load address %s\n",
		    $fit_addr, length($fit), $load);
}

my $spl = substr($u472, 0, $pos);
substr($spl, $sym{_binman_payload_prop_size}, 8) = pack("Q<", length($fit));
my $c471 = slurp("$ub/u-boot-rockchip-usb471.bin");

spew("$out-usb471.bin", $c471);
spew("$out-usb472.bin", $spl . $fit);
rkboot("$out-loader.bin", $c471, $spl . $fit);
printf "%s-usb471.bin, -usb472.bin (FIT 0x%x bytes at 0x%x, staged at %s)\n", $out,
	length($fit), $pos, $fit_addr;
printf "%s-loader.bin (rkdeveloptool db, chip %s)\n", $out, $chip;

# RC4 with the BootROM key, one keystream over the whole buffer (as xrock
# --rc4 on; rkdeveloptool's own pack truncates the length to 16 bits)
sub rc4 {
	my ($data) = @_;
	my @k = (124, 78, 3, 4, 85, 5, 9, 7, 45, 44, 123, 56, 23, 13, 23, 17);
	my @s = (0 .. 255);
	my $j = 0;

	for my $i (0 .. 255) {
		$j = ($j + $s[$i] + $k[$i & 15]) & 0xff;
		@s[$i, $j] = @s[$j, $i];
	}
	my @d = unpack("C*", $data);
	my $i = 0;
	$j = 0;
	for my $x (0 .. $#d) {
		$i = ($i + 1) & 0xff;
		$j = ($j + $s[$i]) & 0xff;
		@s[$i, $j] = @s[$j, $i];
		$d[$x] ^= $s[($s[$i] + $s[$j]) & 0xff];
	}
	return pack("C*", @d);
}

# CRC32 of rkdeveloptool and boot_merger: polynomial 0x04c10db7, MSB first,
# initial value 0, no final xor
sub rkcrc32 {
	my ($data) = @_;
	my @tab = map {
		my $c = $_ << 24;
		for (1 .. 8) {
			$c = ($c & 0x80000000) ? (($c << 1) ^ 0x04c10db7) : ($c << 1);
			$c &= 0xffffffff;
		}
		$c;
	} 0 .. 255;
	my $crc = 0;

	$crc = (($crc << 8) & 0xffffffff) ^ $tab[($crc >> 24) ^ $_] for unpack("C*", $data);
	return $crc;
}

# Rockchip loader with one 0x471 and one 0x472 entry; entry data RC4 coded
# and padded to 2 KiB, rkdeveloptool sends it as is
sub rkboot {
	my ($file, $d471, $d472) = @_;
	my ($hsz, $esz) = (102, 57);
	my @t = localtime;
	my @ent = (['usb471', 1, $d471, 10], ['usb472', 2, $d472, 10]);
	my $off = $hsz + $esz * @ent;
	my ($ents, $data) = ('', '');

	for (@ent) {
		my ($name, $type, $d, $delay) = @$_;

		$d .= "\0" x ((2048 - length($d) % 2048) % 2048);
		$ents .= pack("C V a40 V V V", $esz, $type, pack("v*", map { ord } split //, $name),
			      $off + length($data), length($d), $delay);
		$data .= rc4($d);
	}
	my $hdr = pack("V v V V v C C C C C V C V C C V C C V C C C a57",
		       0x544f4f42, $hsz, 0x0100, 0x01030000,
		       $t[5] + 1900, $t[4] + 1, $t[3], $t[2], $t[1], $t[0],
		       $chip_code,
		       1, $hsz, $esz, 1, $hsz + $esz, $esz, 0, $hsz + 2 * $esz, $esz,
		       0, 0, "");			# no signature, RC4 on
	my $img = $hdr . $ents . $data;

	spew($file, $img . pack("V", rkcrc32($img)));
}
