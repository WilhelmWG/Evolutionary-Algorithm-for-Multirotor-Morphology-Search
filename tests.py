import utils as ut
import numpy as np

def test_transformation_matrix():
    R = np.array([[0,1,0],[1,0,0],[0,0,-1]])
    t = np.array([2,2,2])
    T = ut.transformation_matrix(R,t)
    print(T)


def test_unskew():
    R = np.array([[0,1,2],[0,0,3],[0,0,0]])
    print(ut.unskew(R))
def main():
    test_transformation_matrix()
    test_unskew()

if __name__ == "__main__":
    main()