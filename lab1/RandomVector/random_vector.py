import random
class RandomVector:
    def __init__(self, size, max_val=1.0):
        self.rv = [max_val*random.random() for index in range(size)]
        
    def mean(self):
        
        # initialize
        total = 0
        length = 0

        # find sum and length
        for number in self.rv:
            total += number
            length += 1
    
        return total/length
        

    def max(self):
        m = self.rv[0]
        for number in self.rv:
            if number > m:
                m = number
        return m

    def min(self):
        m = self.rv[0]
        for number in self.rv:
            if number < m:
                m = number
        return m
    
    def print(self):
        for number in self.rv:
            print(number,end=" ")
        else:
            print("")

    def print_histogram(self, bins):
        # number range
        ran = self.max() - self.min()
        
        # bin width
        bin_width = ran/bins

        # histogram edges
        edges = [bin_width*x + self.min() for x in range(bins+1)]

        # initialize bin counts
        bin_counts = [0] * bins

        # find bin for each number
        current_bin = 0
        for number in self.rv:
            while True:
                if (number >= edges[current_bin]) & (number <= edges[current_bin+1]):
                    bin_counts[current_bin] += 1
                    break
                current_bin +=1
                
                # edge case (due to machine precision with final edge value)
                if current_bin == bins:
                    bin_counts[current_bin-1] += 1
                    break  
                    
            # reset
            current_bin = 0
        
        # find maximum bin count
        max_bin = bin_counts[0]
        for bin_count in bin_counts:
            if bin_count > max_bin:
                max_bin = bin_count

        # generate histogram
        current_bin_count = max_bin
        while current_bin_count > 0:
            for bin_count in bin_counts:
                if bin_count >= current_bin_count:
                    print("***",end=" ")
                else:
                    print("   ",end=" ")
            else:
                print("")
            current_bin_count += -1