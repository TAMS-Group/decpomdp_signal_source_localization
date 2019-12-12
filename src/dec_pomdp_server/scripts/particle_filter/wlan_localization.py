from problem import Problem
import numpy as np
from scipy.stats import rice
from scipy.special import jv as besseli
from tools import compute_distance

class WLANLocalization(Problem):
    def __init__(self, locations, neighbours, Ptx=18.0, Gtx=1.5, Ltx=0.0, Grx=1.5, Lrx=0.0, v=2.4e6, mu=4.0, sigma=20.0):
        super(WLANLocalization, self).__init__(locations, neighbours)
        self.RSS_base = Ptx + Gtx - Ltx + Grx - Lrx + 27.55 - 20*np.log10(v)
        self.sigma = sigma
        self.rice_b = mu / sigma

    def rss_noiseless(self, distances):
        return self.RSS_base - 20.0 * np.log10(distances)

    def single_likelihood(self, source_locations, l, z):
        np
        d = compute_distance(source_locations.reshape(-1,2), l.reshape(-1,2))
        rss = self.rss_noiseless(d)
        fading = rss-z
        if np.all(fading < 0):
            # ALL source locations have neg. fading, ignore this measurement
            p = np.ones(fading.shape)
        else:
            p = rice.pdf(fading, self.rice_b, scale=self.sigma)
        return p

    def likelihood(self, source_locations, position1, observation1):
        l = np.array(position1)
        z = np.array(observation1)
        return self.single_likelihood(source_locations, l, z)
