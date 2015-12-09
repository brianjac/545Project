#!/usr/bin/perl

use strict;
use warnings;

sub mem_to_coe {
    #get file name
    my $file; 
    $file = $ARGV[0];

    if (!($file =~ /(.*)\.mem/)) {
	die "input file is not a .mem file!\n";
    }
    my $name = $1;

    my $mem_file;
    #open file
    open($mem_file, "<", $file) or die "cannot open $file: $! $?";

    #open new coe file
    my $coe = join('', $name, ".coe");
    my $coe_file;

    open($coe_file, ">", $coe) or die "cannot open $coe_file: $! $?";

    #write radix lines into coe file
    print $coe_file "; initialization file for $file\n";
    print $coe_file "memory_initialization_radix = 16;\n";
    print $coe_file "memory_initialization_vector = \n";
    
    #parse file
    my $line;
    my $cnt = 0;
    my $new_line = '';
    while ($line = <$mem_file>) {
	    #if first character is not an @, skip
	    #if first character is an @, get thing
	    my $first_char = substr $line, 0, 1;
	    if (!($first_char eq "@")) {
	        next;
	    }
	    #if the first character is an @, get the byte code for that line
	    $line =~ /(\S{8})$/;
	    my $byte_code = $1;

	    #my $byte_1 = substr $byte_code, 0, 2;
	    #my $byte_2 = substr $byte_code, 2, 2;
	    #my $byte_3 = substr $byte_code, 4, 2;
	    #my $byte_4 = substr $byte_code, 6, 2;

	    #my $new_line = join(' ', $byte_1, $byte_2, $byte_3, $byte_4);
	    $new_line = join('',$byte_code,$new_line);
	    #print "newline now=$new_line\n";
	    $cnt=$cnt+1;
	    if ($cnt==4) {
  	    print $coe_file "$new_line\r\n";
  	    $new_line = '';
  	    $cnt=0;
  	  }
    }
    print $coe_file ";\r\n";
    return;
}

mem_to_coe();
