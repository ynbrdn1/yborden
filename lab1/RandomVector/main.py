from random_vector import RandomVector
def main():
    # test case: provided
    # Create a RandomVector of size 20
    rv = RandomVector(20)
    # Print all values
    rv.print()
    # Print statistics
    print(f"Mean: {rv.mean():.6f}")
    print(f"Min: {rv.min():.6f}")
    print(f"Max: {rv.max():.6f}")
    # Print histogram with 8 bins
    print("Histogram:")
    rv.print_histogram(8)

    # test case: different max value and vector length, also use smaller amount of bins
    rv1 = RandomVector(10, max_val = 100)
    rv1.print()
    print(f"Mean: {rv1.mean():.6f}")
    print(f"Min: {rv1.min():.6f}")
    print(f"Max: {rv1.max():.6f}")
    # print histogram with 3 bins
    print("Histogram:")
    rv1.print_histogram(3)

    # test case: number of bins greater than vector length
    print("Histogram with more bins:")
    rv1.print_histogram(15)

if __name__ == "__main__":
    main()