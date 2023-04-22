import numpy as np


def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    H = np.matmul(np.transpose(AA),BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.matmul(Vt.T, U.T)

    if np.linalg.det(R) < 0:
        print("Reflection detected")
        Vt[2, :] *= -1
        R = np.matmul(Vt.T,U.T)

    t = -np.matmul(R, centroid_A) + centroid_B
    return R, t


if __name__=='__main__':
    # [x, z, y]
    a = np.array(
        [
            [0, 0, 6],
            [0, 0, 12],
            [12, 0, 12],
            [12, 0, 6],
            [12, 0, 0],
            [0, 0, 0]
        ]
    )
    b = np.array(
        [
            [-0.745434, -0.000771, 6.046419],
            [0.367101, -0.785586, 11.946209],
            [12.220451, -1.666759, 9.414896],
            [11.059564, -0.889234, 3.515226],
            [9.930631, -0.184235, -2.304462],
            [-1.882269, 0.747539, 0.055195]
        ]
    )
    zero = np.array([0, 0, 0])
    for i in range(a.shape[0]):
        if (a[i] == zero).all():
            A = np.sqrt(np.sum(np.power(a[:-1,:]-np.tile(a[i], (a.shape[0] - 1, 1)), 2), axis=1))
            B = np.sqrt(np.sum(np.power(b[:-1,:]-np.tile(b[i], (b.shape[0] - 1, 1)), 2), axis=1))
            # print(np.power(b[:-1,:]-np.tile(b[i], (a.shape[0] - 1, 1)), 2))
    # A = np.sqrt(np.sum(np.power(a[:-1,:], 2), axis=1))
    # B = np.sqrt(np.sum(np.power(b[:-1,:], 2), axis=1))
    s = np.mean(A/B)
    c = b*s
    r, t = rigid_transform_3D(a, c)
    str = "\t\t\t\tcout << (" + str(r[0][0]) + " + " + str(-t[0]) + "*CPos(0, 3) + " + str(r[1][0]) + " + (" + str(-t[1]) + "*CPos(1, 3) + " + str(r[2][0]) + " + (" + str(-t[2]) + "*CPos(2, 3))*" + str(s) + " << \" \";\n"    \
        + "\t\t\t\tcout << (" + str(r[0][1]) + " + " + str(-t[0]) + "*CPos(0, 3) + " + str(r[1][1]) + " + (" + str(-t[1]) + "*CPos(1, 3) + " + str(r[2][1]) + " + (" + str(-t[2]) + "*CPos(2, 3))*" + str(s) + " << \" \";\n"    \
        + "\t\t\t\tcout << (" + str(r[0][2]) + " + " + str(-t[0]) + "*CPos(0, 3) + " + str(r[1][2]) + " + (" + str(-t[1]) + "*CPos(1, 3) + " + str(r[2][2]) + " + (" + str(-t[2]) + "*CPos(2, 3))*" + str(s) + " << \" \";"
    f = open('test2.txt', 'w+')
    f.write(str)
    f.close()
    print('R:', np.transpose(r))
    print('t:', t)
    print('A/B:', A/B)
    print('s:', s)


    print(str)