INTRODUCTION
===============================================================================
This is my implementation of the 5 point algorithm to calculate an essential matrix based on the paper

H. Li and R. Hartley, "Five-point motion estimation made easy", icpr, 2006

Given 5 matching points, it'll find the correct essential matrix (out of a max of 10 possibilities) and
also return the 3x4 projection matrix (rotation/translation).


COMPILING
===============================================================================
To compile you need OpenCV 2.x.x or greater

You can compile by typing 'make' or by opening the project in CodeBlocks.


USAGE
===============================================================================
See main.cpp for usage.


IMPLEMENTATION INFO
===============================================================================
First of I want to say, despite what the title of the paper may say or imply, "Five-point motion estimation made easy"
is NOT EASY, not in C++ anyway, at least not for me.

The algorithm outlined in the paper requires symbolic matrix support. This has been the trickiest and
hardest part to implement in C++ code. There are some free symbolic libraries out there that support C/C++ like GINAC
that I thought could do the job. The problem  was when I needed to calculate the determinant of a 10x10 symbolic matrix.
None of the existing packages could do this in an efficient manner, if at all. I remember one time letting Maxima ran all night
and it still never finished it! This ended up being a big show stopper. After some period of time of searching I found 
some papers which showed a practical way of calculating the determinant of symbolic matrices:

    "Symbolic Determinants: Calculating the Degree", http://www.cs.tamu.edu/academics/tr/tamu-cs-tr-2005-7-1
    "Multivariate Determinants Through Univariate Interpolation", http://www.cs.tamu.edu/academics/tr/tamu-cs-tr-2005-7-2

My final solution required me to code up a simple symbolic matrix class and a Maxima/PHP script to generate C++ code.
I'm quite proud of the Maxima/PHP hack, which generated the Mblock.h file. The scripts I used can be found in the "scripts"
directory. To generate the Mblock.h header do this:

1. Run maxima
2. load("5point.mac"); quit();
3. ./5point-script.php > Mblock.h
