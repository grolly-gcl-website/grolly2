#!/bin/sh



echo "Bash version ${BASH_VERSION}..."
for i in {0..100}
  do
	./autopair.sh 20:13:06:19:15:11
	if [ $? = 0 ]; then
		let "succ += 1"
	else
		let "failed += 1"
        fi
	
     echo -e "\rSuccess: $succ and Failed: $failed" > test_result
 done
