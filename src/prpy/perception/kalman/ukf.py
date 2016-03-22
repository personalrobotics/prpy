#!/usr/bin/env python

# Copyright (c) 2016, Carnegie Mellon University
# All rights reserved.
# Authors: Gilwoo Lee <gilwool@andrew.cmu.edu>

# Unscented Kalman Filter

import numpy as np
import logging

logger = logging.getLogger(__name__)


def get_sigma_points(mu, cov, Lambda=None, alpha=1e-2, beta=2, k=1000):
    """
    Returns 2n+1 points and their weights to be processed 
    for UKF where n is the dimension of mu.

    @param mu: mean
    @param cov: covariance
    @param Lambda: scaling parameter
    @return X: Sigma points (2n+1) to be evaluated.
    @return Wm: Weights to be used for mean estimate.
    @return Wc: Weights to be used for cov estimate.
    """
    import scipy.linalg as LA

    n = np.shape(mu)[0]

    if not Lambda:
        Lambda = alpha**2.0*(n+k) - n

    X = np.matrix(np.empty(shape=(n, 2*n + 1)))
    X[:, 0] = mu
    W = list()

    msr = np.matrix(LA.sqrtm(np.matrix((n+Lambda)*cov)))

    print "msr"
    print msr
    for i in range(n):
        X[:, 2*i + 1] = mu + msr[:, i]
        X[:, 2*i + 2] = mu - msr[:, i]
        W += [1/(2*(n + Lambda)), 1/(2*(n + Lambda))]

    # Insert w0 for Wm and Wc.
    Wm = np.array([Lambda/(n + Lambda)] + W) 
    Wc = np.array([Lambda/(n + Lambda) + 1 - alpha**2 + beta] + W)

    # making weights non-negative; is this correct?
    if Wm.min()<0:
        print "Old Wm\n", Wm
        Wm = Wm + abs(2*Wm.min())
        Wm = Wm/sum(Wm)
        print "Wm\n", Wm
    if Wc.min()<0:
        Wc = Wc + abs(2*Wc.min())
        Wc = Wc/sum(Wc)
        print "Wc", Wc


    return X, Wm, Wc


def weighted_cov(mu, X, weights):
    dim = mu.shape[0]
    total = np.matrix(np.zeros(shape=(dim, dim)))
    for i in range(X.shape[1]):
        total += weights[i]*np.dot(np.matrix(X[:, i] - mu),
                                   np.transpose(np.matrix(X[:, i] - mu)))

    return total


def weighted_xz_cov(mu, X, mu_z, Z, weights):
    dim = (mu.shape[0], mu_z.shape[0])
    total = np.matrix(np.zeros(shape=dim))
    for i in range(X.shape[1]):
        total += weights[i]*np.dot(np.matrix(X[:, i] - mu),
                                   np.transpose(np.matrix(Z[:, i] - mu_z)))

    return total


def unscented_kalman_filter(mu, cov, u, z, R, Q, process, measure):
    """ Recursive function for UKF update.
    @param mu: mean at t-1
    @param cov: covariance at t-1
    @param u: action at t
    @param z: observation at t
    @param R: process uncertainty
    @param Q: sensing uncertainty
    @param process: function that propagates current state to next state.
    @param measure: function that returns observation.
    @returns new_mu: updated mean at t
    @returns new_cov: updated covariacne at t
    """

    # Get sigma points 
    X, Wm, Wc = get_sigma_points(mu, cov)

    # Propagate the sigma points to next state.
    X_processed = process(u, X)
    mu_predicted = np.average(X_processed, axis=1, weights=Wm)
    cov_predicted = weighted_cov(mu_predicted, X_processed, Wc) + R

    logger.info("X old\n %s", str(X))
    logger.info("X_processed\n %s", str(X_processed))
    logger.info("mu_predicted %s", str(mu_predicted))

    # Get sigma points from the processed distribution.
    X_next, Wm_next, Wc_next = get_sigma_points(mu_predicted, cov_predicted)

    logger.info("X_next based on prediction\n %s", X_next)

    # Predict observation distribution.
    Z = measure(X_next)
    z_mean = np.average(Z, axis=1, weights=Wm_next)
    S = weighted_cov(z_mean, Z, Wc_next) + Q
    XZ_cov = weighted_xz_cov(mu_predicted, X_next, z_mean, Z, Wc_next)

    logger.info("z\n%s", str(z))
    logger.info("z mean\n%s", str(z_mean))
    logger.info("Z\n%s", str(Z[:,:5]))

    import IPython
    IPython.embed()

    from numpy.linalg import inv
    Kalman_gain = XZ_cov*inv(S)
    new_mu = mu_predicted + Kalman_gain*(z - z_mean)
    new_cov = cov_predicted - Kalman_gain*S*np.transpose(Kalman_gain)

    return new_mu, new_cov


if __name__ == "__main__":
    def process(u, X):
        return X + np.matrix([[1], [1]])

    def measure(X):
        return X

    mu = np.matrix([[0], [0]])
    cov = np.matrix(np.identity(2))
    u = None
    z = np.matrix([[1], [1]])
    R = 0
    Q = np.matrix(np.identity(2))

    for i in range(4):
        new_mu, new_cov = unscented_kalman_filter(mu, cov, u, z,
                                                  R, Q, process, measure)
        print "z\n", z
        print "new mu\n", new_mu
        print "new cov\n", new_cov
        print
        mu = new_mu
        cov = new_cov
        z = z+1
