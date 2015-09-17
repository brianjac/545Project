#! /usr/bin/perl
use strict;
use warnings;
use Getopt::Std;

# Options
our $opt_v = 0; # Verbose
our $opt_h = 0; # Help
our $opt_d = 0; # Dry Run
our $opt_f = 0; # Fail on Match Not Found
our $opt_i = ".";

# Prototypes
sub printv;
sub fixIncludes;
sub printHelp;


# Parse Command Line
getopts('vhdfi:');

if ($opt_h) {
    printHelp();
    exit(0);
}

# Fix the files
printv "==========================$/",
       "Parsing Directory Tree for $opt_i$/",
       "==========================$/";
my @files = readDirectory($opt_i);

for my $file (@files) {
    if ($file =~ /\.vh?$/) {
	fixIncludes($file);
    }
}


# Recursively reads a directory to list all available files
sub readDirectory {
    my $directory = shift;

    my @files;

    printv "Looking at directory $directory$/";

    local *DIR;
    opendir (DIR, $directory) or 
	die "[Error] Failed to open '$directory'!$/$!$/";

    while (my $file = readdir(DIR)) {
	my $filepath = "$directory/$file";

	# Ignore hidden files
	if ($file =~ /^\./) {
	    printv "\tIgnoring $file$/";
	    next;
	}
	
	if (-f $filepath) {
	    printv "\t$filepath$/";
	    push @files, $filepath;
	}

	elsif (-d $filepath) {
	    printv "\tDir: $filepath$/";
	    push @files,readDirectory($filepath);
	}
    }

    closedir(DIR);

    return @files;
}

# Reads in a file and replaces all the `include lines with
# correct local filepaths.
sub fixIncludes {
    my $file = shift;
    my $fh;
    my $new_data;

    printv "==========================$/",
           "Fixing file: $file$/",
           "==========================$/";

    open ($fh, "<", $file) or 
	die "[Error] Could not open file $file.$/$!$/";

    while (my $line = <$fh>) {
	if ($line =~ /`include "(.*)"/) {
	    my $short_name = $1;
	    my @matches;

	    # Filter the list of all files to find 
            # those that end with $short_name.
	    @matches = grep(/$short_name$/,@files);

	    # Warn on multiple matches and exit.
	    if (scalar @matches > 1) {
		die "[Error] Found multiple matches for the file $short_name in $file.$/";
	    }
	    
	    # Warn on no matches and exit.
	    if (scalar @matches < 1) {
		if ($opt_f) {
		    die "[Error] Found no matches for the file $short_name included in $file.$/";
		}
		else {
		    print "[Warning] Found no matches for the file $short_name included in $file.$/";
		    next;
		}
	    }

	    my $match = pop @matches;
	    $match = ($match =~ s/^.\///r);

	    if ($opt_d) {
		printv "[Dry Run] Would replace line $/\t$line";
		printv "with line $/\t`include \"$match\"$/";
	    }
	    else {
		printv "Replaced line $/\t$line";
		$line = "`include \"$match\"\n";
		printv "with line $/\t$line$/";
	    }
	}
	$new_data .= $line;
    }

    if (!$opt_d) {
	open ($fh, ">", $file) or 
	    die "[Error] Could not write to file $file.$/$!$/";

	print $fh $new_data;
    }
}

# Verbose Print
sub printv {
    my @args = @_;

    if ($opt_v) {
	print @args;
    }
}

# Print the command line options/sample usage.
sub printHelp {
    print
	"Verilog Include Fixer",$/,
	"\tBrian Jacobs (brian\@brianjaco.bz), September 17, 2015",$/,
	$/,
	"Summary:",$/,
	"\tThis program reads in a directory of verilog files (and",$/,
	"all of their subdirectories) and replaces `include directives'",$/,
	"files with filepaths relative to the top directory.", $/,
	$/,
	"Command line Options:",$/,
	"\t-v: (V)erbose; Prints debugging information.",$/,
	"\t-h: (H)elp; Displays this message.",$/,
	"\t-d: (D)ry run; Doesn't actually change any files. Useful in",$/,
	"\t\tcombination with -v to see what changes would be made.",$/,
	"\t-f: (F)ail on no match; Stops the run and exits if any",$/,
	"\t\tinclude cannot be satisfied.",$/,
	"\t-i: (I)nput directory; The directory to be cleaned up.",$/,
	$/,
	"Example Usage:",$/,
	"\tperl IncludeFixer.pl -vd -i verilog",$/;
}