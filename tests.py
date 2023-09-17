import utils as ut
import numpy as np

def test_transformation_matrix():
    R = np.array([[0,1,0],[1,0,0],[0,0,-1]])
    t = np.array([2,2,2])
    T = ut.transformation_matrix(R,t)
    print(T)

def main():
    test_transformation_matrix()

if __name__ == "__main__":
    main()