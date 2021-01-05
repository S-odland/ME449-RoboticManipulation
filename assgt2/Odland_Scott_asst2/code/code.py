import modern_robotics as mr 
import numpy as np

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):

    H1 = .1519
    H2 = .08535
    W1 = .11235
    W2 = .0819
    L1 = .24365
    L2 = .21325
 
    B1 = np.array([0,1,0,W1+W2,0,L1+L2])
    B2 = np.array([0,0,1,H2,-L1-L2, 0])
    B3 = np.array([0,0,1,H2,-L2,0])
    B4 = np.array([0,0,1,H2,0,0])
    B5 = np.array([0,-1,0,-W2,0,0])
    B6 = np.array([0,0,1,0,0,0])
    Blist = np.array([B1,B2,B3,B4,B5,B6])
    Blist = Blist.T
 
    # thetalist0 = np.array([1.707,-1.578,0,-1.514,-0.032,1.514]) 
    thetalist0 = np.array([-0.641,-0.419,-0.676,0,0.87,0])

    M = np.array([[-1, 0, 0, 0.45671838521957], 
                  [0, 0, 1, 0.19418559968472], 
                  [0, 1, 0, 0.07377764582634],
                  [0,0,0,1]])
    
    T = np.array([[0.88665968179703, -0.43939724564552, -0.14409957826138, 0.13819913566113], 
                         [0.41775861382484, 0.6275218129158, 0.6570343375206, 0.036001659929752], 
                         [-0.19827343523502, -0.64276474714279, 0.73996025323868, 0.60956245660782],
                         [0,0,0,1]])
                  
    # M = np.array([[-1, 0, 0, L1+L2], 
    #               [0, 0, 1, W1+W2+0.2816], 
    #               [0, 1, 0, H1-H2],
    #               [0,0,0,1]])
    
    # T = np.array([[ 0, 0,-1,-0.34298],
    #             [ 0,-1, 0,-0.098685],
    #             [-1, 0, 0, 1.1109],
    #             [ 0, 0, 0, 1]])

    # T = np.array([[0.003,0.104,-0.995,-0.391],
    #               [0.008,-0.995,-0.104,-0.044],
    #               [-1,-0.007,-0.004,0.699],
    #               [0,0,0,1]])
    eomg = 0.01
    ev = 0.01

    ## done with initial values

    thetalist = np.array(thetalist0).copy()
    iter_thetas = np.array(thetalist.copy())
    i = 0
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, thetalist)), T)))
    
    eomg_act = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
    ev_act = np.linalg.norm([Vb[3], Vb[4], Vb[5]])

    err = eomg_act > eomg or ev_act > ev
    
    T_act = mr.FKinBody(M, Blist, thetalist)

    print(" Iteration %d:\n\n \
        Joint Vector:\n \
        %6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f\n\n \
        SE(3) End-Effector Configuration:\n \
        %6.4f %6.4f %6.4f %6.4f\n \
        %6.4f %6.4f %6.4f %6.4f\n \
        %6.4f %6.4f %6.4f %6.4f\n \
        %6.4f %6.4f %6.4f %6.4f\n\n \
        Error Twist Vb:\n \
        %6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f\n\n \
        Angular Error Magnitude || omega_b ||: %6.5f\n \
        Linear Error Magnitude || v_b ||: %6.5f\n" % (i,thetalist[0],thetalist[1],thetalist[2],thetalist[3],thetalist[4],thetalist[5],
        T_act[0,0],T_act[0,1],T_act[0,2],T_act[0,3],T_act[1,0],T_act[1,1],T_act[1,2],T_act[1,3],T_act[2,0],T_act[2,1],T_act[2,2],T_act[2,3],
        T_act[3,0],T_act[3,1],T_act[3,2],T_act[3,3],Vb[0],Vb[1],Vb[2],Vb[3],Vb[4],Vb[5],eomg_act,ev_act))

    while err:

        i = i + 1
        thetalist = thetalist + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, thetalist)), Vb)

        iter_thetas = np.vstack([iter_thetas,thetalist])

        Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, thetalist)), T)))

        T_act = mr.FKinBody(M, Blist, thetalist)

        eomg_act = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
        ev_act = np.linalg.norm([Vb[3], Vb[4], Vb[5]])

        err = eomg_act > eomg or ev_act > ev

        print(" Iteration %d:\n\n \
                Joint Vector:\n \
                %6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f\n\n \
                SE(3) End-Effector Configuration:\n \
                %6.4f %6.4f %6.4f %6.4f\n \
                %6.4f %6.4f %6.4f %6.4f\n \
                %6.4f %6.4f %6.4f %6.4f\n \
                %6.4f %6.4f %6.4f %6.4f\n\n \
                Error Twist Vb:\n \
                %6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f\n\n \
                Angular Error Magnitude || omega_b ||: %6.5f\n \
                Linear Error Magnitude || v_b ||: %6.5f\n" % (i,thetalist[0],thetalist[1],thetalist[2],thetalist[3],thetalist[4],thetalist[5],
                T_act[0,0],T_act[0,1],T_act[0,2],T_act[0,3],T_act[1,0],T_act[1,1],T_act[1,2],T_act[1,3],T_act[2,0],T_act[2,1],T_act[2,2],T_act[2,3],
                T_act[3,0],T_act[3,1],T_act[3,2],T_act[3,3],Vb[0],Vb[1],Vb[2],Vb[3],Vb[4],Vb[5],eomg_act,ev_act))

    f = open("output.csv", "w") 

    for i in range(len(iter_thetas)):
        output = "%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f\n" % (iter_thetas[i,0], iter_thetas[i,1], iter_thetas[i,2], 
                                                                        iter_thetas[i,3], iter_thetas[i,4], iter_thetas[i,5])
        f.write(output)

    f.close()

    return iter_thetas, not err

if __name__ == '__main__':
    B1 = np.array([0,1,0,0.45532432198524, 0.080698430538177, -0.39312356710434])
    B2 = np.array([0,0,1,0.45535561442375, 0.085098803043365, -0.28145736455917])
    B3 = np.array([0,0,1,0.21170553565025, 0.08503133058548, -0.28145730495453])
    B4 = np.array([0,0,1,-0.0015476942062378, 0.085083961486816, -0.28145805001259])
    B5 = np.array([0,-1,0,-0.0015476942062378, 0, -0.28145796060562])
    B6 = np.array([0,0,1,-0.0015476942062378, 0, -0.28145796060562])
    Blist = np.array([B1,B2,B3,B4,B5,B6])
    Blist = Blist.T

    thetalist0 = np.array([1.707,-1.578,0,-1.514,-0.032,1.514]) 
    M = np.array([[-1, 0, 0, 0.45545893907547], 
                  [0, 0, 1, 0.39318609237671], 
                  [0, 1, 0, 0.073756009340286],
                  [0,0,0,1]])
    # T = np.array([[ 0, 0,-1,-0.34298],
    #             [ 0,-1, 0,-0.098685],
    #             [-1, 0, 0, 1.1109],
    #             [ 0, 0, 0, 1]])

    T = np.array([[0,0,-1,-0.4],[0,-1,0,0],[-1,0,0,0.7],[0,0,0,1]])
    eomg = 0.001
    ev = 0.0001
    IKinBodyIterates(Blist,M,T,thetalist0,eomg,ev)