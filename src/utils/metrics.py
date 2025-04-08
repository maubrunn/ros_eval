
from scipy.signal import welch
from scipy.special import rel_entr
import numpy as np


'''
    Each function takes:
    x: np-array
    y: np-array

    Returns:
    metric as a float (or None)
'''

def pearson_correlation(x, y):
    corr = np.corrcoef(x, y)[0, 1]
    if np.isnan(corr):
        return None
    else:
        print(f"Correlation: {corr}")
        return corr
    
def cross_correlation(x, y):
    corr = np.correlate(x, y, mode='full')
    print("Max Cross Correlation: ", corr.max())
    return corr

def kld(x, y, fs=40):
    # Compute PSD using Welch's method
    _, psd1 = welch(x, fs=fs)
    _, psd2 = welch(y, fs=fs)

    # Normalize the PSDs to ensure they sum to 1
    psd1_norm = psd1 / np.sum(psd1)
    psd2_norm = psd2 / np.sum(psd2)

    # Compute the Kullback-Leibler Divergence
    kld = np.sum(rel_entr(psd1_norm, psd2_norm))
    if np.isnan(kld):
        return None
    print("KLD: ", kld)
    return kld