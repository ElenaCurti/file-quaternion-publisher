permutations=(
# ELENA
"0 1 2 3"
"0 1 3 2"
"0 2 1 3"
"0 2 3 1"
"0 3 1 2"
"0 3 2 1"
"1 0 2 3"
"1 0 3 2"
"1 2 0 3"
"1 2 3 0"
"1 3 0 2"
"1 3 2 0"
"2 0 1 3"
# CASTOR
"2 0 3 1"
"2 1 0 3"
"2 1 3 0"
"2 3 0 1"
"2 3 1 0"
"3 0 1 2"
"3 0 2 1"
"3 1 0 2"
"3 1 2 0"
"3 2 0 1"
"3 2 1 0"

    # Add more permutations as needed
)

# Loop through permutations and run the C++ program with each permutation
for perm in "${permutations[@]}"; do
    echo "Running with permutation: $perm"
    ros2 run mostra_img mostra_img $perm
done
