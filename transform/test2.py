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
    a = np.array(
        [
            [12, 0, 0],
            [12, 0, 6],
            [0, 0, 6],
            [0, 0, 0]
        ]
    )
    b = np.array(
        [
            [12.149437, 0.098873, 0.328044],
            [12.045532, 0.131505, 6.301020],
            [-0.202044, 0.061753, 6.120294],
            [-0.002855, 0.001278, 0.004739]
        ]
    )
    A = np.sqrt(np.sum(np.power(a[:-1,:], 2), axis=1))
    B = np.sqrt(np.sum(np.power(b[:-1,:], 2), axis=1))
    s = np.mean(A/B)
    c = b*s
    r, t = rigid_transform_3D(a, c)
    print('R:', np.transpose(r))
    print('t:', t)
    print('A/B:', A/B)
    print('s:', s)
