# def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):

#     ## below are the initial values for assignment 2

#     thetalist0 = np.array([-3.712,0.683,-1.659,-5.109,-0.683,4.458])  
#     M = np.array([[-1,0,0,.817],[0,0,1,.191],[0,1,0,-.006],[0,0,0,1]])  
#     Blist = np.array([[0,0,0,0,0,0],[1,0,0,0,-1,0],[0,1,1,1,0,1],[.191,.095,.095,.095,-.082,0],[0,-.817,-.392,0,0,0],[.817,0,0,0,0,0]]) 
#     T = np.array([[0,1,0,-0.5],[0,0,-1,0.1],[-1,0,0,0.1],[0,0,0,1]])
#     eomg = 0.001
#     ev = 0.0001

#     ## done with initial values

#     thetalist = np.array(thetalist0).copy()
#     iter_thetas = np.array(thetalist.copy())
#     i = 0
#     Vb = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, thetalist)), T)))
    
#     eomg_act = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
#     ev_act = np.linalg.norm([Vb[3], Vb[4], Vb[5]])

#     err = eomg_act > eomg or ev_act > ev
    
#     T_act = FKinBody(M, Blist, thetalist)

#     print(" Iteration %d:\n\n \
#         Joint Vector:\n \
#         %6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f\n\n \
#         SE(3) End-Effector Configuration:\n \
#         %6.4f %6.4f %6.4f %6.4f\n \
#         %6.4f %6.4f %6.4f %6.4f\n \
#         %6.4f %6.4f %6.4f %6.4f\n \
#         %6.4f %6.4f %6.4f %6.4f\n\n \
#         Error Twist Vb:\n \
#         %6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f\n\n \
#         Angular Error Magnitude || omega_b ||: %6.5f\n \
#         Linear Error Magnitude || v_b ||: %6.5f\n" % (i,thetalist[0],thetalist[1],thetalist[2],thetalist[3],thetalist[4],thetalist[5],
#         T_act[0,0],T_act[0,1],T_act[0,2],T_act[0,3],T_act[1,0],T_act[1,1],T_act[1,2],T_act[1,3],T_act[2,0],T_act[2,1],T_act[2,2],T_act[2,3],
#         T_act[3,0],T_act[3,1],T_act[3,2],T_act[3,3],Vb[0],Vb[1],Vb[2],Vb[3],Vb[4],Vb[5],eomg_act,ev_act))

#     while err:

#         i = i + 1
#         thetalist = thetalist + np.dot(np.linalg.pinv(JacobianBody(Blist, thetalist)), Vb)

#         iter_thetas = np.vstack([iter_thetas,thetalist])

#         Vb = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, thetalist)), T)))

#         T_act = FKinBody(M, Blist, thetalist)

#         eomg_act = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
#         ev_act = np.linalg.norm([Vb[3], Vb[4], Vb[5]])

#         err = eomg_act > eomg or ev_act > ev

#         print(" Iteration %d:\n\n \
#                 Joint Vector:\n \
#                 %6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f\n\n \
#                 SE(3) End-Effector Configuration:\n \
#                 %6.4f %6.4f %6.4f %6.4f\n \
#                 %6.4f %6.4f %6.4f %6.4f\n \
#                 %6.4f %6.4f %6.4f %6.4f\n \
#                 %6.4f %6.4f %6.4f %6.4f\n\n \
#                 Error Twist Vb:\n \
#                 %6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f\n\n \
#                 Angular Error Magnitude || omega_b ||: %6.5f\n \
#                 Linear Error Magnitude || v_b ||: %6.5f\n" % (i,thetalist[0],thetalist[1],thetalist[2],thetalist[3],thetalist[4],thetalist[5],
#                 T_act[0,0],T_act[0,1],T_act[0,2],T_act[0,3],T_act[1,0],T_act[1,1],T_act[1,2],T_act[1,3],T_act[2,0],T_act[2,1],T_act[2,2],T_act[2,3],
#                 T_act[3,0],T_act[3,1],T_act[3,2],T_act[3,3],Vb[0],Vb[1],Vb[2],Vb[3],Vb[4],Vb[5],eomg_act,ev_act))

#     f = open("output.csv", "w") 

#     for i in range(len(iter_thetas)):
#         output = "%7.6f,%7.6f,%7.6f,%7.6f,%7.6f,%7.6f\n" % (iter_thetas[i,0], iter_thetas[i,1], iter_thetas[i,2], 
#                                                                         iter_thetas[i,3], iter_thetas[i,4], iter_thetas[i,5])
#         f.write(output)

#     f.close()

#     return iter_thetas, not err