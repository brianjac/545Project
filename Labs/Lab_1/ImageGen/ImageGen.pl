#! /usr/bin/perl
use strict;

my @pixels = <>;

# Remove Header
shift(@pixels); # Filetype identifier
shift(@pixels); # Size identifier
shift(@pixels); # 

# Map from 0-255 to 0-1
my @bits = map{chomp; $_ >= 128?1:0;}@pixels;

# Convert bit clusters into numbers.
while (scalar @bits != 0) {
    my $array_length = scalar @bits;
    my $binary_string = sprintf "%s"x8,
      @bits[0],@bits[1],@bits[2],@bits[3],
      @bits[4],@bits[5],@bits[6],@bits[7];
    
    # Print the actual character
    print chr(oct("0b".$binary_string));

    @bits = @bits[8..($array_length-1)];
}