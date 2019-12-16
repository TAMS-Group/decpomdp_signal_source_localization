from problem import Problem
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import rice
from scipy.special import jv as besseli
from tools import compute_distance


# Defines the source localization measurement model as first described in
# Atanasov, Le Ny, Pappas: "Distributed algorithms for stochastic source seeking
# with mobile robot networks"
# Journal of Dynamic Systems, Measurement, and Control 137 (3)
#
# and used in
#
# Lauri, Pajarinen, Peters: "Information Gathering in Discrete and
# Continuous-state Decentralized POMDPs by Policy Graph Improvement"
class SourceSeekingRSS(Problem):
	def __init__(self, locations, neighbours, Ptx=18.0, Gtx=1.5, Ltx=0.0,
					Grx=1.5, Lrx=0.0, v=2.4e6, mu=4.0, sigma=20.0):
		super(SourceSeekingRSS, self).__init__(locations, neighbours)
		self.RSS_base = Ptx + Gtx - Ltx + Grx - Lrx + 27.55 - 20*np.log10(v)
		self.sigma = sigma
		self.rice_b = mu / sigma

	def rss_noiseless(self, distances):
		return self.RSS_base - 20.0 * np.log10(distances)

	def single_likelihood(self, source_locations, l, z):
		d = compute_distance(source_locations.reshape(-1,2), self.locations[l].reshape(-1,2))
		rss = self.rss_noiseless(d)
		fading = rss-z
		print("l is: " + str(l)+" z is: " + str(z) + " Location is: " + str(self.locations[l]) + " D is: " + str(d) + " rss is: " + str(rss) + " fading is: " + str(fading) )
		if np.all(fading < 0):
			# ALL source locations have neg. fading, ignore this measurement
			p = np.ones(fading.shape)
		else:
			p = rice.pdf(fading, self.rice_b, scale=self.sigma)
		print("probabilities: " + str(p))
		return p

	def sample(self, source_locations, l):
		d = compute_distance(source_locations.reshape(-1,2), self.locations[l].reshape(-1,2))
		rss = self.rss_noiseless(d)
		num_samples = d.shape[0]
		fading = rice.rvs(self.rice_b, scale=self.sigma, size=(num_samples,))
		return rss - fading

	def likelihood(self, source_locations, l0, l1, z0, z1):
		return self.single_likelihood(source_locations, l0, z0) * self.single_likelihood(source_locations, l1, z1)

	def sample_joint_observation(self, source_locations, l0, l1):
		return self.sample(source_locations, l0), self.sample(source_locations, l1)

# Visualize the model
def main():
	Ptx = 18.0 # dBm
	Gtx = 1.5 # dBi
	Ltx = 0.0 # dB
	Grx = 1.5 # dBi
	Lrx = 0.0 # dB
	v = 2.4e6 # frequency in Hz

	# RSS without noise as function of distance
	d = np.arange(0.01, 40.0, step=0.1)
	Lfs = -27.55 + 20.0*np.log10(v) + 20.0*np.log10(d)
	RSS = Ptx + Gtx - Ltx + Grx - Lrx - Lfs

	# Rice distr.
	mu = 4.0 # dB
	sigma = 30.0 # dB
	b = mu / sigma # shape parameter for SciPy Rice

	# Plot the RSS mean and noise Rice PDF
	x = np.linspace(0.0, 100.0, 200)
	pr = rice.pdf(x, b, scale=sigma)

	# Manual PDF
	pm = (x/sigma**2)*np.exp(-(x**2+mu**2)/(2*sigma**2))*besseli(0,x*mu/sigma**2)

	# Rice CDF limits
	rlow = rice.ppf(0.05, b, scale=sigma)
	rmean = rice.mean(b, scale=sigma)
	rhi = rice.ppf(0.95, b, scale=sigma)

	fig, ax = plt.subplots(3,1)
	ax[0].plot(d, RSS, 'r--', label='Noise-free')
	ax[0].plot(d, RSS-rmean, label='Mean')
	ax[0].fill_between(d, RSS-rlow, RSS-rhi, alpha=0.2, label='5...95 percent')

	levels = [-110.0, -125.0]
	for level in levels:
		ax[0].plot((d[0],d[-1]), (level, level), lw=2, color='k', linestyle='--')
	# Draw some samples for illustration
	idx = 100
	num_samples = 100
	noise_samples = rice.rvs(b, scale=sigma, size=(num_samples,))
	rss_samples = RSS[idx] - noise_samples
	ax[0].plot(np.repeat(d[100], num_samples), rss_samples, 'ro', label='Random samples')

	ax[0].legend()
	ax[0].grid()



	ax[1].plot(x, pr, 'k', lw=5, alpha=0.6, label='Rician pdf')
	# ax[1].plot(x, pm, 'r--', lw=5, alpha=0.6, label='rice pdf')
	ax[1].grid()
	ax[1].legend()

	# Draw the PDFs for each observation according to the levels
	cdf0 = rice.cdf(RSS-levels[0], b, scale=sigma)
	cdf1 = rice.cdf(RSS-levels[1], b, scale=sigma)
	# ax[2].plot(d, cdf0, 'b', lw=5)
	# ax[2].plot(d, cdf1, 'r', lw=5)
	# ax[2].plot(d, cdf0-cdf1, 'k', lw=5)

	p0 = cdf0
	p1 = cdf1 - cdf0
	p2 = 1.0 - cdf1

	ax[2].plot(d, p0, 'k', lw=5, label='HIGH')
	ax[2].plot(d, p1, 'b', lw=5, label='MED')
	ax[2].plot(d, p2, 'r', lw=5, label='LOW')
	# ax[2].plot(d,p0+p1+p2,'g',label='sum')
	ax[2].grid()
	ax[2].legend()

	plt.show()


	print('---- Values for the C++ code ---')
	probs = np.linspace(0.0, 0.9999, 300)
	q = rice.ppf(probs, b, scale=sigma)
	cdf = rice.cdf(q, b, scale=sigma)

	print('static std::vector<double> losses{' + ', '.join(str(v) for v in q.tolist()) + '};')
	print('static std::vector<double> cdf{' + ', '.join(str(v) for v in cdf.tolist()) + '};')


if __name__ == "__main__":
	main()
