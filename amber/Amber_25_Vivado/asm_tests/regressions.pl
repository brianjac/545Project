#! /usr/bin/perl
use strict;
use warnings;

sub test_asm;

########################
# Statistics Variables #
########################

# Test passage statistics
my $TOTAL_TESTS = 0;
my $TESTS_PASSED = 0;
my $TESTS_FAILED = 0;

# Total Timing Statistics
my $START_TIME;
my $END_TIME;

my @months = qw( Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec );

my $VIVADO_TESTS_PATH = 
    "/home/DRRA/545Project/amber/Amber_25_Vivado/".
    "Amber_25/Amber_25/Amber_25.srcs/sources_1/imports/vlog/tests";

my $HTML_HEADER =
'<html>
  <head>
    <title>DRRA Regression Results</title>
  </head>
  <body>
    <h1>Test Results</h1>
    <div style="width:775px;text-align:justify;text-justify:inter-word;">If a test runs longer than the testbench is configured to simulate, it times out and returns its own name. In these cases, the test is highlighted in yellow. Passing tests are highlighted in green. Failing tests are highlighted in red. The tests below are all referred to during simulation as \'add\' because they\'re given the filenames associated with the add test. This makes automatically running the tests easier. 
    </div>
    <br>
    <table border="1" style="width:775px">
    <tr>
     <th>Test Name</th><th>Time Completed</th><th>Result</th>
    </tr>';

my $TABLE_BODY = "";

#################
# Program Logic #
#################
opendir (DIR, ".") or die $!;

# Iterate over the tests to produce the test table.
$START_TIME = time();
while (my $file = readdir(DIR)) {
    if ($file =~ /(.*)\.S$/) {
	$TOTAL_TESTS += 1;
	$TABLE_BODY .= test_asm($1);
    }
}
$END_TIME = time();

my $HTML_FOOTER =
  "</table>
   <br>
   Total testing time: ".($END_TIME - $START_TIME)." | Tests Passed: $TESTS_PASSED/$TOTAL_TESTS
  </body>
</html>";

print "$HTML_HEADER$TABLE_BODY$HTML_FOOTER";

print STDERR "Total testing time: ",($END_TIME - $START_TIME)," seconds";

# Argument is a test name.
sub test_asm {
    my $test_name = shift;
    my $return_string = "";

    print STDERR "Testing $test_name...";

    `rm $VIVADO_TESTS_PATH/add.mem 2> /dev/null`;
    `rm $VIVADO_TESTS_PATH/add_memparams32.v 2> /dev/null`;
    `rm $VIVADO_TESTS_PATH/add_memparams128.v 2> /dev/null`;
    `make all TEST=$test_name`;

    # Copy everything that could be relevant into the vivado project path.
    `cp $test_name.mem $VIVADO_TESTS_PATH/add.mem`;
    `cp ${test_name}_memparams32.v $VIVADO_TESTS_PATH/add_memparams32.v`;
    `cp ${test_name}_memparams128.v $VIVADO_TESTS_PATH/add_memparams128.v`;

    # Run the test
    my $results = `cat simulate.tcl | vivado -mode tcl`;

    # Clean the current directory.
    `make clean`;
    `rm vivado*.jou vivado*.log`;

    # Craft the return string.
    (my $sec,my $min,my $hour,my $mday,
     my $mon,my $year,my $wday,my $yday,my $isdst) = localtime();
    $year += 1900; # Year is indexed starting at 1900 for whatever ungodly reason.
    $return_string .= "<tr><td>$test_name</td><td>$hour:$min:$sec - $mday $months[$mon], $year</td>";

    if ($results =~ /(Failed.*?)\n/) {
	if (!($test_name eq "fail")) {
	    $TESTS_FAILED += 1;
	}
	else {
	    $TESTS_FAILED += -1;
	}
	$return_string .= "<td bgcolor=\"\#FF9999\">$1</td></tr>$/";
    }
    elsif ($results =~ /(Passed.*?)\n/) {
	$TESTS_PASSED += 1;
	$return_string .= "<td bgcolor=\"\#99FF99\">$1</td></tr>$/";
    }
    else {
	$return_string .= "<td bgcolor=\"#FFFF99\">$test_name timed out</td></tr>";
    }

    print STDERR "Done!",$/;
    
    return $return_string;
}
