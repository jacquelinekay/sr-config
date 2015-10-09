#!/usr/bin/perl

@lines = <>;
$text = join("", @lines);

$text =~ s/[^\S\n]+c.move_hand\((.+?)\)\s+rospy.sleep\(([\.\d]+)\)/{'angles': $1, 'interpolation_time': $2},/g;



print $text;
