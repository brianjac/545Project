#! /usr/bin/perl
use warnings;
use strict;

##############
# Parameters #
##############

# This value must be taken from fonts_list.txt, modulo capitalization.
my $font_name = 'helvetica';

# Width and height of each character
my $font_width = 10;
my $font_height = 12;

#  Color depth is the number of bits per pixel.
# 1 is preferred, but if you want antialiasing,
# set it to something that isn't 1.
my $color_depth = 1;

# The location that temporary files are written
my $image_cache = "img_cache";
my $hex_cache = "hex_cache";
my $verilog_filename = "ASCII2Bitmap.sv";

# The list of characters to generate.
#my $alphabet = "abcdefghijklmnopqrstuvwxyz.";
my $alphabet = 
    q{0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz};

###################
# Program Staging #
###################

# Ensure the caches exist
if (!-d $image_cache) {
    mkdir $image_cache or die "Error creating directory: $1";
}

if (!-d $hex_cache) {
    mkdir $hex_cache or die "Error creating directory: $1";
}

# Generate modified coordinates
my $l_width = $font_width;
my $l_height = $font_height * 1.25;
my $l_position = $l_height * 0.75;

###########################################
# Generate the individual character files #
###########################################

for my $char (split//,$alphabet) {
    
    # Generate a character file
    my $command = 
	"convert -size ${l_width}x${l_height} ".
	"-pointsize $font_height xc:white ".
	"-draw 'text 0,$l_position \"$char\"' ".
	"-compress none ".
	"-depth $color_depth ". 
	"$image_cache/$char.pgm";
    `$command`;

}

##########################################################
# Convert the character files into bitmap hex line files #
##########################################################

for my $char (split//,$alphabet) {
    my $filename = "$image_cache/$char.pgm";

    open (my $fh, "<", $filename)
	or die "Could not open file '$filename'.";

    my $file_nonewline = "";
    my $char_binary = "";

    # Ignore the first three lines, which are header 
    my $ignore = 3;

    while (<$fh>) {
	chomp;
	if ($ignore-- <= 0) {
	    $file_nonewline .= $_." ";
	}
    }

    for (split/\s+/,$file_nonewline) {
	$char_binary .= ($_ eq "255") ? 
	    "1" : "0" ;
    }

    my @char_split = $char_binary =~ m/(?=(.{4}))/g;

    my $char_hex = join "",(map{sprintf('%X', oct("0b$_"))}@char_split);

    open (my $fh_out, ">", "$hex_cache/$char.hex")
	or die "Could not open file '$filename'.";

    print $fh_out $char_hex;

    close $fh;
    close $fh_out;
}

######################################################
# Read the character files to create ASCII2Bitmap.sv #
######################################################

my $verilog = "";

open (my $verilog_fh, ">", $verilog_filename)
    or die "Could not open file '$verilog_filename'.";

# The width of the verilog array.
my $v_size = ($l_width * $l_height) - 1;
my $v_size_p1 = ($l_width * $l_height);

my $verilog_1 = qq{ 
module ASCII2Bitmap
   (input logic  [7:0]  char,
    output logic [$v_size:0]     bitmap);

    always_comb begin
      case (char)
};

for my $char (split//,$alphabet) {
    my $filename = "$hex_cache/$char.hex";

    open (my $fh, "<", $filename)
	or die "Could not open file '$filename'.";

    my $file = <$fh>;
    my $hex_string = sprintf("%X",ord($char));

    $verilog .= "\t// $char := ".ord($char)." [".sprintf("0x%X",ord($char))."]$/";
    $verilog .= "\t8'h$hex_string : bitmap = ${v_size_p1}'h$file;$/";

    close $fh;
}


my $verilog_2 = qq{
      endcase
    end
endmodule : ASCII2Bitmap
};

print $verilog_fh  $verilog_1,$verilog,$verilog_2;
