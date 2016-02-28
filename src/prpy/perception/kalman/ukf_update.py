#!/usr/bin/env python

# Copyright (c) 2016, Carnegie Mellon University
# All rights reserved.
# Authors: Gilwoo Lee <gilwool@andrew.cmu.edu>

# Process, measure, and ukf-update object pose 
# from observed feature point through camera

import numpy as np


def process(u, X):
    """ Since we assume object is stable,
    X' = X.
    If we are moving the object,
    some other internal process should be defined.
    """
    return X


def measure(x, P, T, height=2):
    """ Measurement of x through camera.
    @param x: Object pose in (x,y,theta)
    @param P: Camera matrix (3x4)
    @param T: Offset of feature w.r.t object frame
              T*x is 4x1 homogeneous 3d position
    @return Z:= 3x1 projected feature point on image screen
    """
    Z = np.matrix(np.zeros(shape=(2, x.shape[1])))

    for i in range(x.shape[1]):
        xi = np.array(x[:, i]).tolist()

        # 3D feature point pose
        X = np.matrix(np.identity(4))
        t = xi[2][0]
        X[0:2, 0:2] = np.matrix([[np.cos(t), -np.sin(t)],
                                 [np.sin(t), np.cos(t)]])
        X[:, 3] = np.matrix([[xi[0][0]], [xi[1][0]], [height], [1]])
        F = np.dot(T, X)
        W = np.matrix([[0], [0], [0], [1]])

        # 3D position
        Y = F*W

        # Projection to image screen
        Y = np.dot(P, Y)
        z = Y/Y[2]
        Z[:, i] = z[0:2]

    return Z


def update(mu, cov, P, T, z, height, Q=np.matrix(np.identity(2))):
    """
    Given current estimate (mu, cov) of obj in SE(2)
    and detected feature point z in im screen,
    use UKF to upate mu and cov.

    @param mu: (x,y,theta) object pose
    @param cov: 3x3 covariance matrix
    @param P: camera matrix
    @param T: offset from obj frame to feature point
    @param z: observed (u,v) of feature point on image screen
    @param height: known height of object
    @param Q: measurement noise

    @returns new_mu, new_cov

    """

    from ukf import unscented_kalman_filter
    return unscented_kalman_filter(mu, cov, None, z, 0, Q,
                                   process,
                                   lambda x: measure(x, P, T))


if __name__ == "__main__":
    T = np.matrix(np.identity(4))
    P = np.matrix(np.zeros(shape=(3, 4)))
    P[0:3, 0:3] = np.identity(3)

    from ukf import unscented_kalman_filter
    x = 4
    y = 2
    theta = np.pi/2
    mu = np.matrix([[x], [y], [theta]])
    cov = np.matrix(np.identity(3))
    R = 0
    Q = np.matrix(np.identity(2))

    z = np.matrix([[3], [2]])
    Z = measure(mu, P, T)

    for i in range(20):
        mu_new, cov_new = unscented_kalman_filter(mu, cov, None, z, R, Q,
                                                  process,
                                                  lambda x: measure(x, P, T))

        print "old"
        print mu
        print cov
        print "new"
        print mu_new
        print cov_new

        mu = mu_new
        cov = cov_new
