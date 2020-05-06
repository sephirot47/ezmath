#!/bin/bash

# Go to root dir
cd "$(dirname "$0")"
cd .. 

# For each code file
echo ""
echo "Formatting files... ======"
for f in $(find src/* -type f | grep -E "\.h$|\.tcc$|\.cpp$")
do
	# Format file inplace
	clang-format --assume-filename=.clang-format -i $f
	echo "  Formatted '$f'" 
done

echo "=========================="
echo ""
