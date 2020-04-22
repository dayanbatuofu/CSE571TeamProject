# This code is for CSE 571 Team Project, contributors are as below:
# Siddhartha Cheruvu 
# Aaron Arul Maria John
# Yi Chen
# Lei Zhang
#
# This algorithm of LPA* is based on the following paper:
# "D* Lite"
# https://www.aaai.org/Papers/AAAI/2002/AAAI02-072.pdf


class Queue:
    def __init__(self):
        self.size = 0
        self.UMapToK1 = dict()  # maps u to key tuple
        self.K1MapToU = dict()  # maps k1 to u
        self.minKey = None
        self.minCount = 0

    #def size(self):
        #return self.size

    def min_state(self):
        return self.minKey, self.minCount

    def insert(self, u, calculateKey):
        print('k1, k2~~~~~~~~~~~~~~~~')
        k1, k2 = calculateKey
        if u in self.UMapToK1:
            self.removeU(u)  # if u is pushed twice, update it to the new value

        self.UMapToK1[u] = (k1, k2)
        if not self.K1MapToU.get(k1):
            self.K1MapToU[k1] = []
        self.K1MapToU[k1].append(u)
        self.size += 1
        print(self.size)
        if self.minKey is None or k1 < self.minKey:  # find out the top key when using U.pop()
            self.minKey = k1
            self.minCount = 1
        elif k1 == self.minKey:
            self.minCount += 1

    def removeU(self, u):
        if u in self.UMapToK1:
            # clean up the data first
            old_k1, old_k2 = self.UMapToK1[u]
            self.size -= 1
            del self.UMapToK1[u] #remove the u from dict UMapToK1
            self.K1MapToU[old_k1].remove(u)
            if len(self.K1MapToU[old_k1]) == 0:
                del self.K1MapToU[old_k1]  # remove the k1 from dict K1MapToU

            # can be expensive [O(n)] to recompute the min, so be smart about it
            if old_k1 == self.minKey:
                self.minCount -= 1
                if self.minCount == 0:
                    self.computeMin()  # only compute if current minKey is exhausted

    def computeMin(self):
        if self.size == 0:
            self.minCount = 0
            self.minKey = None
        else:
            k1List = self.K1MapToU.keys()[:]
            self.minKey = min(k1List)
            self.minCount = k1List.count(self.minKey)

    def topKey(self):
        if self.size <= 0:
            return None  # nothing to pop

        # find and break the ties
        uList = self.K1MapToU[self.minKey]
        u_kTopList = []
        for u in uList:
            k1, k2 = self.UMapToK1[u] #k1 is the same for all the u
            u_kTopList.append((u, k1, k2))
        u_kTopList.sort(key=lambda x: x[2])  # sort the list by k2 to find the top key tuple
        print(u_kTopList)
        u_kTop = u_kTopList[0]

        return u_kTop

    def pop(self):
        u_kTop = self.topKey()
        if u_kTop is not None:
            self.removeU(u_kTop[0])  # remove the u and top key tuple
        return u_kTop[0]


