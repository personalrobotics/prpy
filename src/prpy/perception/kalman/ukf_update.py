#!/usr/bin/env python

# Copyright (c) 2016, Carnegie Mellon University
# All rights reserved.
# Authors: Gilwoo Lee <gilwool@andrew.cmu.edu>

# Process, measure, and ukf-update object pose 
# from observed feature point through camera

import numpy


def process(u, X):
    """ Since we assume object is stable,
    X' = X.
    If we are moving the object,
    some other internal process should be defined.
    """
    return X


def convert_to_kinbody_pose(se2_pose, height):
    """ Takes [theta, x, y].
    Returns 4x4 object transform.
    """
    r = se2_pose[0]
    x = se2_pose[1]
    y = se2_pose[2]

    obj_transform = numpy.identity(4)
    obj_transform[:3, :3] = numpy.array(
                                [[numpy.cos(r), -numpy.sin(r), 0.],
                                 [numpy.sin(r),  numpy.cos(r), 0.],
                                 [0., 0., 1.]])
    obj_transform[:, 3] = numpy.array([x, y, height, 1])
    obj_transform = numpy.matrix(obj_transform)
    return obj_transform

def project_marker_to_im_screen(P, marker_pose):
    """ Returns (u,v,1) on image screen """ 
    projection = numpy.dot(P, marker_pose[:, 3])
    
    for i in range(3):
        projection[i] /= projection[2]
    return projection


def measure(x, P, frame_offset, kinbody_offset, height=0.5):
    """ Measurement of x through camera.
    @param x: Object pose in (x,y,theta)
    @param P: Camera matrix (3x4)
    @param frame_offset: offset between april-tag detection frame and world frame
    @param kinbody_offset: offset between kinbody and feature
    @return Z:= 3x1 projected feature point on image screen
    """
    Z = numpy.matrix(numpy.zeros(shape=(2, x.shape[1])))

    for i in range(x.shape[1]):
        # object se2 pose
        xi = numpy.array(x[:, i].transpose())[0]
        # convert to se(3)
        obj_transform = convert_to_kinbody_pose(xi, height)
        # expected marker pose
        expected_marker_pose = numpy.dot(numpy.dot(numpy.linalg.inv(frame_offset),
                                                   obj_transform),
                                         numpy.linalg.inv(kinbody_offset))
        expected_projection = project_marker_to_im_screen(P, expected_marker_pose)

        Z[:, i] = expected_projection[0:2]

    return Z


def update(mu, cov, P, frame_offset, kinbody_offset, z, 
           height=0.5, Q=numpy.matrix(numpy.identity(2))):
    """
    Given current estimate (mu, cov) of obj in SE(2)
    and detected feature point z in im screen,
    use UKF to upate mu and cov.

    @param mu: (x,y,theta) object pose
    @param cov: 3x3 covariance matrix
    @param P: camera matrix
    @param frame_offset: offset between april-tag detection frame and world frame
    @param kinbody_offset: offset between kinbody and feature
    @param z: observed (u,v) of feature point on image screen
    @param height: known height of object
    @param Q: measurement noise

    @returns new_mu, new_cov

    """

    from ukf import unscented_kalman_filter
    return unscented_kalman_filter(mu, cov, None, z, 0, Q,
                                   process,
                                   lambda x: measure(x, P, frame_offset, 
                                                     kinbody_offset, height)
                                   )


if __name__ == "__main__":
    T = numpy.matrix(numpy.identity(4))
    P = numpy.matrix(numpy.zeros(shape=(3, 4)))
    P[0:3, 0:3] = numpy.identity(3)

    from ukf import unscented_kalman_filter
    x = 4
    y = 2
    theta = numpy.pi/2
    mu = numpy.matrix([[x], [y], [theta]])
    cov = numpy.matrix(numpy.identity(3))
    R = 0
    Q = numpy.matrix(numpy.identity(2))

    z = numpy.matrix([[3], [2]])
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
