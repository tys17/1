import numpy as np
import pdb

class Resampling:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 4.3]
    """

    def __init__(self):
        """
        TODO : Initialize resampling process parameters here
        """

    def multinomial_sampler(self, X_bar):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """

        """
        TODO : Add your code here
        """

        return X_bar_resampled

    def low_variance_sampler(self, X_bar):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """

        """
        TODO : Add your code here
        """
        X_bar_resampled = np.ndarray([0,4])

        W = X_bar[:,3].sum()
        n, _ = X_bar.shape

        r = np.random.uniform(0,W)

        mean = W/n

        cur_pos = X_bar[0, 3]

        i = 0

        for m in range(n):
            bar = r + m*mean
            while bar > cur_pos:
                i = (i+1)%n
                cur_pos = (cur_pos+X_bar[i, 3])%W
            X_bar_resampled = np.append(X_bar_resampled, [X_bar[i, :]], axis=0)

        # cnt = mean
        # w_i = X_bar[0, 3]

        # for i in range(n):
        #     if w_cnt == 0:
        #         w_cnt = X_bar[i, 3]
        #     if cnt == 0:
        #         cnt = mean
        #
        #     if w_cnt == cnt:
        #         w_cnt = 0
        #         cnt = mean
        #         X_bar_resampled = np.append(X_bar_resampled, [X_bar[i,:]], axis = 0)
        #         continue
        #     elif w_cnt > cnt:
        #         w_cnt -= cnt
        #         cnt = mean
        #         X_bar_resampled = np.append(X_bar_resampled, [X_bar[i,:]], axis = 0)
        #         continue
        #     else: # w_cnt < cnt
        #         w_cnt = 0
        #         cnt -= w_cnt
        #         continue

        
        return X_bar_resampled

if __name__ == "__main__":
    pass