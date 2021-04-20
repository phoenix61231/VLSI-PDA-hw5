make clean
make
/usr/bin/time -p ../bin/hw5 ../testcase/ibm01.modified.txt ../output/ibm01.result
../verifier/verify ../testcase/ibm01.modified.txt ../output/ibm01.result
/usr/bin/time -p ../bin/hw5 ../testcase/ibm04.modified.txt ../output/ibm04.result
../verifier/verify ../testcase/ibm04.modified.txt ../output/ibm04.result
