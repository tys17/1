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

        W = (10**X_bar[:,3]).sum()
        n, _ = X_bar.shape

        r = np.random.uniform(0,W)

        mean = W/n*2

        cur_pos = 10**X_bar[0, 3]

        i = 0
        cnt = 0

        prev_i = i
        prev_pos = X_bar[0, :]
        prev_neg = X_bar[0, :]
        for m in range(n):
            if cnt>n:
                break

            bar = (r + m*mean)%W
            while bar > cur_pos:
                i = (i+1)%n
                cur_pos = (cur_pos+10**X_bar[i, 3])
            cur_pos = cur_pos % W
            if prev_i == i:
                store_x = prev_pos[0]
                store_y = prev_pos[1]
                store_ang_pos = prev_pos[2]
                store_ang_neg = prev_neg[2]
                wt = np.log10(10**prev_pos[3]/2)

                rand1 = np.array(
                    [X_bar[i, 0]+np.random.normal(-40, 40),X_bar[i, 1]+np.random.normal(-40, 40), store_ang_pos + np.pi/4 + np.random.normal(-np.pi/10,np.pi/10),wt])
                X_bar_resampled = np.append(X_bar_resampled, [rand1], axis=0)
                prev_pos = rand1

                rand2 = np.array(
                    [X_bar[i, 0]+np.random.normal(-40, 40),X_bar[i, 1]+np.random.normal(-40, 40), store_ang_neg - np.pi / 4 + np.random.normal(-np.pi / 10, np.pi / 10), wt])
                X_bar_resampled = np.append(X_bar_resampled, [rand2], axis=0)
                prev_neg = rand2

                cnt += 2
            else:
                X_bar_resampled = np.append(X_bar_resampled, [X_bar[i, :]], axis=0)
                prev_pos = X_bar[i, :]
                prev_neg = X_bar[i, :]
                cnt += 1
            prev_i = i


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