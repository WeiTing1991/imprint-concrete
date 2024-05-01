import numpy as np


class TravelligSaleman():

    def __init__(self, initialT, d_ij, showTourOutput=False):
        # InitialT & d_ij is set to what is passed in.
        self.T = initialT
        self.d_ij = d_ij
        # Set D to be length of tour T
        self.D = self.calculateLengthOfTour(initialT)
        self.showTourOutput = showTourOutput
    
    def calculateLengthOfTour(self, tour):
        sumDistance = 0
        for ind in range(len(tour)):
            v_1 = tour[ind]
        
            #If at the end of tour, use home V
            if(ind == (len(tour) -1 )):
                v_2 = tour[0]
            else:
                v_2 = tour[ind+1]
            
            sumDistance += self.d_ij[v_1][v_2]
        
        return sumDistance
    def runTS(self):
        N = len(self.T)
        # i = 1 , i = i +1 while i<N-3
        for i in range(N-3):
        # j = i +2, j = j+1, while j < N-1
            for j in range(i+2,N-1):
                newT = np.copy(self.T)
                newT[i+1] = self.T[j]
                ind = i + 2 
                for k in range(j-1,i,-1):
                    newT[ind] = self.T[k]
                    ind += 1
                newT_D = self.calculateLengthOfTour(newT)
                
                if(self.showTourOutput == True):
                    print("i: ",i,"j: ",j,"Possible T",newT, "Cost: ", newT_D, "CurrentBestCost:", self.D)
                if(newT_D < self.D):
                    self.T = np.copy(newT)
                    self.D = np.copy(newT_D) 
                    self.runTS()
                iterations = i * j
                print ("iterations: ", iterations, end="\r")