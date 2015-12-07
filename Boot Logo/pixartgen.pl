#! /usr/bin/perl
# Converts images into .COE files.

my $image_name = "TestPattern-4.png";
my $image_name_prefix = "TestPattern-4";
my %color_map = ();
my %backwards_map = ();

my $image_name_quantized = "cache/".$image_name_prefix."-quantized.png";
my $image_name_ppm = "cache/".$image_name_prefix.".ppm";
my $image_name_coe = $image_name_prefix.".coe";

# First, ensure that it has only 16 colors.
`convert $image_name +dither -colors 15 $image_name_quantized`;

# Snag those colors and output them to a file.
my $color_data = `convert $image_name_quantized -verbose info:`;

my @lines = split/\n/,$color_data;

# Get to the color map.
while ($lines[0] !~ /Colormap/) {
    shift @lines;
}
shift @lines;

# While inside the color map, extract colors.
while ($lines[0] !~ /Rendering/) {
    $lines[0] =~ /\s*(.*):\s*\(\s*(.*),\s*(.*),\s*(.*)\) #/;
    $color_map{$1} = [$2,$3,$4];
    $backwards_map{"$2 $3 $4"} = $1;
    shift @lines;
}

# Write the colors to a color file.
my $file_colors = "$image_name-colors.txt";
open($fh_colors,">",$file_colors) or die $!;

for $key (sort {$a <=> $b} keys %color_map) {
    my $red = $color_map{$key}[0];
    my $green = $color_map{$key}[1];
    my $blue = $color_map{$key}[2];
    print $fh_colors "$key -> ($red, $green, $blue)",$/;
}

close $fh_colors;

# Convert the png into a ppm.
`convert $image_name_quantized -compress none -depth 4 $image_name_ppm`;


# Convert the ppm into a .COE
open ($fh_ppm, "<", $image_name_ppm) or die $!;
open ($fh_coe, ">", $image_name_coe) or die $!;

my @ppm_lines = <$fh_ppm>;
my @coe_lines;

# Get rid of the header.
shift @ppm_lines;
shift @ppm_lines;
shift @ppm_lines;
shift @ppm_lines;

my @ppm_pixels = split/\s+/,(join " ",@ppm_lines);

while (@ppm_pixels) {
    my $r = (shift @ppm_pixels);
    my $g = (shift @ppm_pixels);
    my $b = (shift @ppm_pixels);

    push @coe_lines, sprintf("%x",$backwards_map{"$r $g $b"});
}

print $fh_coe "memory_initialization_radix=16;$/";
print $fh_coe "memory_initialization_vector=";
print $fh_coe (join ",\n", @coe_lines);
print $fh_coe ";$/";

close $fh_coe;
close $fh_ppm;
