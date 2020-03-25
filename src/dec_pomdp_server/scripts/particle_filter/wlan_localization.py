#!/usr/bin/env python
from problem import Problem
import rospy
import numpy as np
from scipy.stats import rice
from scipy.special import jv as besseli
from tools import compute_distance

class WLANLocalization(Problem):
    def __init__(self, locations, neighbours, Ptx=12.0, Gtx=1.5, Ltx=0.0, Grx=1.5, Lrx=0.0, v=2.4e9, mu=4.0, sigma=12.551, rice_b=0.009, rice_loc=-7.001):
        super(WLANLocalization, self).__init__(locations, neighbours)
        self.RSS_base = 147.55 - 20*np.log10(v)
        self.sigma = sigma
        self.rice_b = rice_b
        self.rice_loc = rice_loc

    def rss_noiseless(self, distances):
        return self.RSS_base - 20.0 * np.log10(distances)

    def single_likelihood(self, source_locations, l, z):
        d = compute_distance(source_locations.reshape(-1,2), l.reshape(-1,2))
        rss = self.rss_noiseless(d)
        fading = rss-z
        if np.all(fading < 0):
            # ALL source locations have neg. fading, ignore this measurement
            p = np.ones(fading.shape)
        else:
            p = rice.pdf(fading, self.rice_b, loc=self.rice_loc, scale=self.sigma)
        return p

    def likelihood(self, source_locations, position1, observation1):
        l = np.array(position1)
        z = np.array(observation1)
        return self.single_likelihood(source_locations, l, z)
