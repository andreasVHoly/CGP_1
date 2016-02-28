README for CGP Assignment 1
VHLAND002
Andreas von Holy


NOTE:

For some reason one needs to close the program each time you want to load a new model. I tried cleaning vectors etc but still results where wrong if 2 models were called in the same instance of the program.


_________________________________________________________________________
This is also the reason that all tests have their own mesh object

To run the assignment:

1. Navigate inside the cgp1-prep directory.
2. Type 'make'.
3. Hopefully all tests pass and you will see a 100% appear eventually. Ignore all the warnings that will show up on screen.
4. Type './tesselate/tessviewer' to run the program.
5. Then navigate to the File option in the actions tab and open a .stl file.


_________________________________________________________________________
To Test the assignment:

NOTE: Some tests have not been completely filled out. In their TODO comments it explains why they are not completed

1. Navigate inside the cgp1-prep directory.
2. Type 'make'.
3. Hopefully all tests pass and you will see a 100% appear eventually. Ignore all the warnings that will show up on screen.
4. Type './test/tilertest --test "All tests"' to run the tests.
5. See the output to see what values are outputted and to see what tests are passed and why tests fail.
