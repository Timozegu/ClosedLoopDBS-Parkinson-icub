# _ -*- coding: cp1252 -*-
import random
from netpyne import specs, sim, cell
from scipy import signal, integrate
import scipy
import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio
from scipy.signal import argrelmax
import pylab
from GA_params import GA_params
from utils import compute_mfr
import pickle
from nitime.algorithms import multi_taper_psd
import math
import socket


# TODO This is the Kumaravelu model class. Verify if all possible configurations are flexible, i.e., can be configured when a new object is instantiated


# prepare a property object


# create remote driver

# Configuration 1
# bregion AP ML DV
# M1 10 6.5 14.4
# GPi 8 3.5 7.8
# GPe 8 5.2 8.8
# Put 8.5 6.5 11.5
# VL 5.5 3.7 10.5
# VPL 4.5 4.3 9.2
# STN 5.5 3.7 7.6

# Configuration 2
# M1 10 6.5 14.4
# S1 8 5.2 15.6
# Put 8.5 6.5 11.5
# VL 5.5 3.7 10.5
# VPL 4.5 4.3 9.2
# STN 5.5 3.7 7.6


###################### Health / Parkinson ###################
# RS-> StrD1 connections
# GPe-< GPe connections
# Str.mod
#############################################################

class Structure:
    class Spikes:
        def __init__(self):
            self.times = []

    def __init__(self,
                 has_pd=True,
                 dbs=False,
                 t_sim=200000,
                 n_channels=1,
                 seed=1,
                 stim_interval=10000,
                 imax=1e-3
                 ):
        print(has_pd)
        self.stim_interval = stim_interval
        self.imax = imax
        self.stim_c0 = 0
        self.stim_c1 = 0
        self._tt = 0
        self.addedspikes = []
        self.bool = 0
        self.allbeta_mfr = []
        self.allbeta_spikes = []
        self.allallbeta_diff = []
        self.population = []
        self.populationWrist = []
        self.mutation_rate = 0.2
        self.newDBS = 0
        self.allFitness = []
        self.finalvalue = 0
        self.Wrist = 0
        self.child = 0.0
        self.prevchild = 0
        self.alldbs = []
        self.overall_interval_mfr = []
        self.AllWrist = []
        self.flag_first_inter = True
        self.i_interval = 1
        self.betaband_table = []
        self.PDcoeff_table = []
        random.seed(seed)
        self.nelec = 1

        self.pd = has_pd
        self.has_dbs = dbs
        self.dbs = 0
        self.t_sim = t_sim
        self.n_channels = n_channels

        self.netParams = self.buildNetParams()
        self.buildPopulationParameters()
        self.buildCellRules()
        self.buildSynMechParams()
        self.buildCellConnRules()
        self.buildStimParams()
        '''
        self.coeff = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_host = 'localhost'  # Replace with the appropriate IP address or hostname
        self.sender_port = 12345  # Choose a suitable port number

        self.coeff.bind((self.server_host, self.sender_port))

        self.coeff.listen(1)
        print('Waiting for client connection from robot...')

        self.client_socket, self.client_address = self.coeff.accept()
        print('Client robot connected:', self.client_address)
        '''

    def IntervalFunc(self, t):
        print('INTERVAL FUNC - interval n=', self.i_interval)

        sim.gatherData()  # gather spiking data and cell info from each node
        sim.saveData()

        self.interval_spikes()

        self.fuseCortexSpikes()

        self.buildFiringActivity()

        self.compute_mfr()

        self.compute_isis()

        self.compute_PSD()

        self.computeBetaPower()

        self.getPDcoeff()

        # self.send_socket()

        # self.receivesocket()

        if self.has_dbs == True:
            print("UPDATING DBS")
            self.update_dbs()

        print('DBS : ', self.alldbs)
        print('PD coeff : ', self.PDcoeff_table)

        self.i_interval += 1

    def interval_spikes(self):
        spikes = self.extractSpikes()
        self.intervalspikes = spikes

        if not self.flag_first_inter:
            for brain_reg in spikes.keys():
                for neur_id in range(len(self.intervalspikes[brain_reg])):
                    self.intervalspikes[brain_reg][neur_id].times = [t for t in spikes[brain_reg][neur_id].times if
                                                                     t not in self.addedspikes[brain_reg][
                                                                         neur_id].times]
                    self.addedspikes[brain_reg][neur_id].times.extend(self.intervalspikes[brain_reg][neur_id].times)
        else:
            self.flag_first_inter = False
            self.addedspikes = spikes

    def fuseCortexSpikes(self):
        self.spikesCTX = [spk.times for spk in self.intervalspikes['Cor_RS_APs_0']]
        self.spikesCTX += [spk.times for spk in self.intervalspikes['Cor_FSI_APs_0']]

    def buildFiringActivity(self):
        plot = False
        self.firing_rate = np.zeros((len(self.spikesCTX), self.stim_interval))
        for neuron in self.spikesCTX:
            for spike in neuron:
                spike = round(spike - self.i_interval * self.stim_interval)
                self.firing_rate[self.spikesCTX.index(neuron), spike] = 1

        if plot == True:
            plt.imshow(self.firing_rate, aspect='auto', cmap='binary')
            plt.xlabel('Time (ms)')
            plt.ylabel('Cortex neurons')
            plt.title('Spike train per neurons during the last interval')
            plt.show()

    def compute_mfr(self):
        self.interval_mfr = []
        self.added_mfr = []
        for key in self.intervalspikes.keys():
            self.trans_mfr = 0
            self.transadd_mfr = 0
            print(key)
            for i in range(len(self.intervalspikes[key])):
                self.trans_mfr += len(self.intervalspikes[key][i].times)
            self.trans_mfr /= ((self.stim_interval / 1000) * len(self.intervalspikes[key]))
            self.interval_mfr.append(self.trans_mfr)
        for i in range(0, 8):
            self.interval_mfr[i] = round(self.interval_mfr[i], 2)
        self.overall_interval_mfr.append(self.interval_mfr)
        print("Interval MFR : ", self.interval_mfr)

    def compute_isis(self):
        self.ISI_combined = []
        plot = False
        for neuron in self.spikesCTX:
            for ISI in np.diff(neuron):
                self.ISI_combined.append(round(ISI))

        if plot == True:
            hist = np.histogram(self.ISI_combined, max(self.ISI_combined))[0]  # Compute histogram of ISIs,
            plt.bar(np.linspace(0, max(self.ISI_combined), max(self.ISI_combined)), hist, width=1)  # ... and plot it.
            plt.ylim([0, 45])
            plt.xlim([0, 1400])
            plt.xlabel('Interspike Interval (ms)')
            plt.ylabel('Count')
            plt.title('Histogram of ISIs of the cortex')
            plt.show()

        print("Mean ISI : ", np.mean(np.array(self.ISI_combined)))
        print("STD ISI : ", np.std(np.array(self.ISI_combined)))

        print("CV : ", np.std(np.array(self.ISI_combined)) / np.mean(np.array(self.ISI_combined)))

    def compute_PSD(self):
        plot = False
        self.f, PSD_neuron, var = multi_taper_psd(self.firing_rate * 1000, Fs=1000, NW=4)
        PSD_neuron *= 1000 ** 2 / 2
        self.PSD = np.mean(PSD_neuron, axis=0)
        if self.i_interval != 1:
            self.PSDstudy = self.PSD
        print(self.f)
        if plot == True:
            fig, ax = plt.subplots()
            ax.plot(self.f[0:500], self.PSD[0:500])  # Plot the planning period,
            ax.axvspan(8, 30, facecolor='lightgreen', alpha=0.3, label="Beta band")
            ax.set_xlabel('Freq (Hz)')  # ... with axes labeled
            ax.set_ylabel('Power (Hz)')
            ax.set_title('Neuron-averaged Power Spectrum Density of spiking data')
            plt.show()

    def computeBetaPower(self):
        self.beta_band = integrate.trapezoid(self.PSD[80:300], x=self.f[80:300]) / integrate.trapezoid(self.PSD[0:500],
                                                                                                       x=self.f[0:500])
        print('Beta Band : ', self.beta_band)
        print('INTERVAL', self.i_interval)
        if self.i_interval != 1:
            print("NOT FIRST ITERATION")
            self.betaband_table.append(self.beta_band)

    def plotBetaPower(self):
        plt.plot(range(0, self.t_sim, self.stim_interval), self.betaband_table)
        plt.title('Evolution of beta power')
        plt.show()

    def getPDcoeff(self):
        self.PDcoeff = round(10 / (1 + math.exp(-150 * (self.beta_band - 0.481))), 2)
        if self.i_interval != 0:
            self.PDcoeff_table.append(self.PDcoeff)
            print('Parkison Disease Coefficient : ', self.PDcoeff)

    def plotPDCoeff(self):
        plt.plot(range(0, self.t_sim, self.stim_interval), self.PDcoeff_table)
        plt.title('Evolution of PD coeff')
        plt.show()

    def send_socket(self):
        self.client_socket.sendall(str(self.PDcoeff).encode())

    def receivesocket(self):
        self.prevWrist = self.Wrist
        self.Wrist = float(self.client_socket.recv(1024).decode())
        self.AllWrist.append(self.Wrist)
        print("RECEIVED DATA :", self.Wrist)

    def update_dbs(self):
        self.upper_thresh = 0.9
        self.lower_thresh = 0.1
        self.alldbs.append(self.dbs)
        if self.PDcoeff > self.upper_thresh:
            self.dbs += (self.PDcoeff - self.upper_thresh) * 0.15
            self.dbs = min(5, self.dbs)
        elif self.PDcoeff < self.lower_thresh:
            self.dbs -= (self.lower_thresh - self.PDcoeff) * 0.22
            self.dbs = max(0, self.dbs)
        else:
            pass

        sim.net.modifyCells({'conds': {'cellModel': 'STN', 'cellType': 'STN'},
                             'secs': {'soma': {'mechs': {'SubTN': {'dbs': self.dbs}}}}})


    def plotElaborate(self):
        time = range(0, self.t_sim,  self.stim_interval)
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
        ax1.plot(time, self.PDcoeff_table, color='b', label='PD state coefficient (a.u.)')
        ax2.plot(time, self.alldbs, color='k', label='DBS current')

        ax1.axhline(y=self.upper_thresh, color='r', linestyle='--', label='upper threshold')
        ax1.axhline(y=self.lower_thresh, color='g', linestyle='--', label='lower threshold')

        for i in range(0, len(time) - 1):
            if self.PDcoeff_table[i] > 1:
                ax2.axvspan(time[i], time[i + 1], facecolor='lightcoral', alpha=0.3)
            elif self.PDcoeff_table[i] < 0.5:
                ax2.axvspan(time[i], time[i + 1], facecolor='lightgreen', alpha=0.3)
            else:
                ax2.axvspan(time[i], time[i + 1], facecolor='lightgrey', alpha=0.3)

        ax1.set_xlim(time[0], time[-1])
        ax1.set_ylim([0, 10])
        # Add labels, legend, and grid
        ax1.set_ylabel('PD')
        ax2.set_ylabel('DBS current (mA/cm^2)')
        ax2.set_xlabel('Time')
        ax1.legend()
        ax2.legend()
        ax1.grid()
        ax2.grid()

        # Show the plot
        plt.tight_layout()
        plt.show()

    def set_genotype(self, genotype):
        ga_params = GA_params()
        genotype = ga_params.transform(genotype)
        self.buildPopulationParameters(n_gpe=int(genotype[6]),
                                       n_gpi=int(genotype[7]),
                                       n_th=int(genotype[8]),
                                       n_strd1=int(genotype[9]),
                                       n_strd2=int(genotype[10]),
                                       n_rs=int(genotype[11]),
                                       n_fsi=int(genotype[12]),
                                       n_stn=int(genotype[13]))
        self.buildCellConnRules()
        # net.buildCellConnRules( stn_gpe = int( genotype[8] ),
        #                        gpe_gpe = int( genotype[9] ),
        #                        stn_gpi = int( genotype[10] ),
        #                        gpe_gpi = int( genotype[11] ),
        #                        strd2_strd2 = int( genotype[12] ),
        #                        strd1_strd1 = int( genotype[13] ),
        #                        rs_fsi = int( genotype[14] ),
        #                        fsi_rs = int( genotype[15] ) )
        self.buildStimParams(amp_th=genotype[0],
                             amp_gpe=genotype[1],
                             amp_gpi=genotype[2])
        for ch in range(self.n_channels):
            self.netParams.cellParams['STN_%d' % ch]['secs']['soma']['mechs']['SubTN']['gkcabar'] = genotype[3]
            self.netParams.cellParams['GPe_%d' % ch]['secs']['soma']['mechs']['GP']['gahp'] = genotype[4]
            self.netParams.cellParams['GPi_%d' % ch]['secs']['soma']['mechs']['GP']['gahp'] = genotype[4]
        self.strConnRules(gsynmod=genotype[5])
        return

    def buildNetParams(self):
        return specs.NetParams()  # object of class NetParams to store the network parameters

    def buildPopulationParameters(self,
                                  n_strd1=10,
                                  n_strd2=10,
                                  n_th=10,
                                  n_gpi=10,
                                  n_gpe=10,
                                  n_rs=10,
                                  n_fsi=10,
                                  n_stn=10):

        self.netParams.sizeX = 7500  # x-dimension (horizontal length) size in um
        self.netParams.sizeY = 8800  # y-dimension (vertical height or cortical depth) size in um
        self.netParams.sizeZ = 5000  # z-dimension (horizontal length) size in um

        # volume occupied by each population can be customized (xRange, yRange and zRange) in um
        # xRange or xnormRange - Range of neuron positions in x-axis (horizontal length), specified 2-element list [min, max].
        # zRange or znormRange - Range of neuron positions in z-axis (horizontal depth)
        # establishing 2000 um as a standard coordinate span

        for ch in range(self.n_channels):
            self.netParams.popParams['StrD1_%d' % ch] = {'cellModel': 'StrD1',
                                                         'cellType': 'StrD1',
                                                         'numCells': n_strd1,
                                                         'xRange': [4000, 6000],
                                                         'yRange': [3900, 5900],
                                                         'zRange': [3000, 5000]}
            self.netParams.popParams['StrD2_%d' % ch] = {'cellModel': 'StrD2',
                                                         'cellType': 'StrD2',
                                                         'numCells': n_strd2,
                                                         'xRange': [4000, 6000],
                                                         'yRange': [3900, 5900],
                                                         'zRange': [3000, 5000]}
            # considering VPL coordinates
            self.netParams.popParams['TH_%d' % ch] = {'cellModel': 'TH',
                                                      'cellType': 'Thal',
                                                      'numCells': n_th,
                                                      'xRange': [0, 2000],
                                                      'yRange': [1600, 3600],
                                                      'zRange': [800, 2800]}
            self.netParams.popParams['GPi_%d' % ch] = {'cellModel': 'GPi',
                                                       'cellType': 'GPi',
                                                       'numCells': n_gpi,
                                                       'xRange': [3500, 5500],
                                                       'yRange': [200, 2200],
                                                       'zRange': [0, 2000]}
            self.netParams.popParams['GPe_%d' % ch] = {'cellModel': 'GPe',
                                                       'cellType': 'GPe',
                                                       'numCells': n_gpe,
                                                       'xRange': [3500, 5500],
                                                       'yRange': [1200, 3200],
                                                       'zRange': [1700, 3700]}
            # considering M1
            self.netParams.popParams['CTX_RS_%d' % ch] = {'cellModel': 'CTX_RS',
                                                          'cellType': 'CTX_RS',
                                                          'numCells': n_rs,
                                                          'xRange': [5500, 7500],
                                                          'yRange': [6800, 8800],
                                                          'zRange': [3000, 5000]}
            self.netParams.popParams['CTX_FSI_%d' % ch] = {'cellModel': 'CTX_FSI',
                                                           'cellType': 'CTX_FSI',
                                                           'numCells': n_fsi,
                                                           'xRange': [5500, 7500],
                                                           'yRange': [6800, 8800],
                                                           'zRange': [3000, 5000]}
            self.netParams.popParams['STN_%d' % ch] = {'cellModel': 'STN',
                                                       'cellType': 'STN',
                                                       'numCells': n_stn,
                                                       'xRange': [1000, 3000],
                                                       'yRange': [0, 2000],
                                                       'zRange': [200, 2200]}

    def buildCellRules(self, **args):
        self.rsCellRules(**args)
        self.fsiCellRules(**args)
        self.strD1CellRules(**args)
        self.strD2CellRules(**args)
        self.thCellRules(**args)
        self.gpiCellRules(**args)
        self.gpeCellRules(**args)
        self.stnCellRules(**args)

    def rsCellRules(self):
        cellRule = {'conds': {'cellModel': 'CTX_RS', 'cellType': 'CTX_RS'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'pointps': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1,
                                            'cm': 1}
        cellRule['secs']['soma']['pointps']['Izhi'] = {'mod': 'Izhi2003b',
                                                       'a': 0.02,
                                                       'b': 0.2,
                                                       'c': -65,
                                                       'd': 8,
                                                       'f': 5,
                                                       'g': 140,
                                                       'thresh': 30}
        cellRule['secs']['soma']['vinit'] = -65
        cellRule['secs']['soma']['threshold'] = 30
        for ch in range(self.n_channels):
            self.netParams.cellParams['CTX_RS_%d' % ch] = cellRule

    def fsiCellRules(self):
        cellRule = {'conds': {'cellModel': 'CTX_FSI', 'cellType': 'CTX_FSI'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'pointps': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1,
                                            'cm': 1}
        cellRule['secs']['soma']['pointps']['Izhi'] = {'mod': 'Izhi2003b',
                                                       'a': 0.1,
                                                       'b': 0.2,
                                                       'c': -65,
                                                       'd': 2,
                                                       'f': 5,
                                                       'g': 140,
                                                       'thresh': 30}
        cellRule['secs']['soma']['vinit'] = -65
        cellRule['secs']['soma']['threshold'] = 30
        for ch in range(self.n_channels):
            self.netParams.cellParams['CTX_FSI_%d' % ch] = cellRule

    def strD1CellRules(self):
        cellRule = {'conds': {'cellModel': 'StrD1', 'cellType': 'StrD1'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['Str'] = {'gmbar': (2.6e-3 - self.pd * 1.1e-3)}
        cellRule['secs']['soma']['vinit'] = random.gauss(-63.8, 5)
        cellRule['secs']['soma']['threshold'] = -10
        for ch in range(self.n_channels):
            self.netParams.cellParams['StrD1_%d' % ch] = cellRule

    def strD2CellRules(self):
        cellRule = {'conds': {'cellModel': 'StrD2', 'cellType': 'StrD2'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['Str'] = {'gmbar': (2.6e-3 - self.pd * 1.1e-3)}
        cellRule['secs']['soma']['vinit'] = random.gauss(-63.8, 5)
        cellRule['secs']['soma']['threshold'] = -10
        for ch in range(self.n_channels):
            self.netParams.cellParams['StrD2_%d' % ch] = cellRule

    def thCellRules(self):
        cellRule = {'conds': {'cellModel': 'TH', 'cellType': 'Thal'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['thalamus'] = {}
        cellRule['secs']['soma']['vinit'] = random.gauss(-62, 5)
        cellRule['secs']['soma']['threshold'] = -10
        for ch in range(self.n_channels):
            self.netParams.cellParams['TH_%d' % ch] = cellRule

    def gpiCellRules(self, gahp=10e-3):
        cellRule = {'conds': {'cellModel': 'GPi', 'cellType': 'GPi'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['GP'] = {'gahp': gahp}
        # cellRule['secs']['GPi']['mechs']['GP'] = {}
        cellRule['secs']['soma']['vinit'] = random.gauss(-62, 5)
        cellRule['secs']['soma']['threshold'] = -10
        for ch in range(self.n_channels):
            self.netParams.cellParams['GPi_%d' % ch] = cellRule

    def gpeCellRules(self, gahp=10e-3):
        cellRule = {'conds': {'cellModel': 'GPe', 'cellType': 'GPe'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['GP'] = {'gahp': gahp}
        # cellRule['secs']['GPe']['mechs']['GP'] = {}
        cellRule['secs']['soma']['vinit'] = random.gauss(-62, 5)
        cellRule['secs']['soma']['threshold'] = -10
        for ch in range(self.n_channels):
            self.netParams.cellParams['GPe_%d' % ch] = cellRule

    def stnCellRules(self, gkcabar=1e-3):
        cellRule = {'conds': {'cellModel': 'STN', 'cellType': 'STN'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['SubTN'] = {'dbs': 0,  # self.dbs,
                                                      'gkcabar': gkcabar}
        cellRule['secs']['soma']['vinit'] = random.gauss(-62, 5)
        cellRule['secs']['soma']['threshold'] = -10
        for ch in range(self.n_channels):
            self.netParams.cellParams['STN_%d' % ch] = cellRule

    def buildSynMechParams(self):
        # TH
        self.netParams.synMechParams['Igith'] = {'mod': 'Exp2Syn',
                                                 'tau1': 5,
                                                 'tau2': 5,
                                                 'e': -85}  # gpi -<th
        # GPe
        self.netParams.synMechParams['Insge,ampa'] = {'mod': 'Exp2Syn',
                                                      'tau1': 0.4,
                                                      'tau2': 2.5,
                                                      'e': 0}  # stn -> gpe
        self.netParams.synMechParams['Insge,nmda'] = {'mod': 'Exp2Syn',
                                                      'tau1': 2,
                                                      'tau2': 67,
                                                      'e': 0}  # stn -> gpe
        self.netParams.synMechParams['Igege'] = {'mod': 'Exp2Syn',
                                                 'tau1': 5,
                                                 'tau2': 5,
                                                 'e': -85}  # gpe -< gpe
        self.netParams.synMechParams['Istrgpe'] = {'mod': 'Exp2Syn',
                                                   'tau1': 5,
                                                   'tau2': 5,
                                                   'e': -85}  # D2 -> gpe
        # GPi
        self.netParams.synMechParams['Igegi'] = {'mod': 'Exp2Syn',
                                                 'tau1': 5,
                                                 'tau2': 5,
                                                 'e': -85}  # gpe -< gp
        self.netParams.synMechParams['Isngi'] = {'mod': 'Exp2Syn',
                                                 'tau1': 5,
                                                 'tau2': 5,
                                                 'e': 0}  # stn -> gpi
        self.netParams.synMechParams['Istrgpi'] = {'mod': 'Exp2Syn',
                                                   'tau1': 5,
                                                   'tau2': 5,
                                                   'e': -85}  # D1 -> gpi
        # STN
        self.netParams.synMechParams['Igesn'] = {'mod': 'Exp2Syn',
                                                 'tau1': 0.4,
                                                 'tau2': 7.7,
                                                 'e': -85}  # gpe -< stn
        self.netParams.synMechParams['Icosn,ampa'] = {'mod': 'Exp2Syn',
                                                      'tau1': 0.5,
                                                      'tau2': 2.49,
                                                      'e': 0}  # ctx -> gpe
        self.netParams.synMechParams['Icosn,nmda'] = {'mod': 'Exp2Syn',
                                                      'tau1': 2,
                                                      'tau2': 90,
                                                      'e': 0}  # ctx -> gpe
        # Str
        self.netParams.synMechParams['Igabadr'] = {'mod': 'Exp2Syn',
                                                   'tau1': 0.1,
                                                   'tau2': 13,
                                                   'e': -80}  # str -< str
        self.netParams.synMechParams['Igabaindr'] = {'mod': 'Exp2Syn',
                                                     'tau1': 0.1,
                                                     'tau2': 13,
                                                     'e': -80}  # str -< str
        self.netParams.synMechParams['Icostr'] = {'mod': 'Exp2Syn',
                                                  'tau1': 5,
                                                  'tau2': 5,
                                                  'e': 0}  # ctx -> str
        # CTX
        self.netParams.synMechParams['Iei'] = {'mod': 'Exp2Syn',
                                               'tau1': 5,
                                               'tau2': 5,
                                               'e': 0}  # rs->fsi
        self.netParams.synMechParams['Iie'] = {'mod': 'Exp2Syn',
                                               'tau1': 5,
                                               'tau2': 5,
                                               'e': -85}  # fsi<-rs
        self.netParams.synMechParams['Ithco'] = {'mod': 'Exp2Syn',
                                                 'tau1': 5,
                                                 'tau2': 5,
                                                 'e': 0}  # th->rs

    def buildCellConnRules(self, **args):
        self.thConnRules(**args)
        self.gpeConnRules(**args)
        self.gpiConnRules(**args)
        self.stnConnRules(**args)
        self.strConnRules(**args)
        self.ctxConnRules(**args)

    def thConnRules(self, **args):
        # GPi-> Th connections
        n_th = self.netParams.popParams['TH_0']['numCells']
        n_gpi = self.netParams.popParams['GPi_0']['numCells']
        n_neurons = max(n_th, n_gpi)
        for ch in range(self.n_channels):
            self.netParams.connParams['GPi->th_%d' % ch] = {
                'preConds': {'pop': 'GPi_%d' % ch}, 'postConds': {'pop': 'TH_%d' % ch},  # GPi-> th
                'connList': [[i % n_gpi, i % n_th] for i in range(n_neurons)],
                'weight': 0.0336e-3,  # synaptic weight (conductance)
                'delay': 5,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Igith'}  # target synaptic mechanism

    def gpeConnRules(self,
                     stn_gpe=2,
                     gpe_gpe=2,
                     **args):
        # STN->GPe connections
        # Two aleatory GPe cells (index i) receive synapse from cells i and i - 1
        n_stn = self.netParams.popParams['STN_0']['numCells']
        n_gpe = self.netParams.popParams['GPe_0']['numCells']
        n_neurons = max(n_stn, n_gpe)
        aux = random.sample(range(n_neurons), stn_gpe)
        connList = [[(x - c) % n_stn, x % n_gpe] for x in aux for c in [1, 0]]
        weight = [random.uniform(0, 0.3) * 0.43e-3 for k in range(len(connList))]
        for ch in range(self.n_channels):
            self.netParams.connParams['STN->GPe_%d' % ch] = {
                'preConds': {'pop': 'STN_%d' % ch}, 'postConds': {'pop': 'GPe_%d' % ch},  # STN-> GPe
                'connList': connList,  # AMPA
                'weight': weight,  # synaptic weight (conductance)
                'delay': 2,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Insge,ampa'}  # target synaptic mechanism
        # STN->GPe connections
        # Two aleatory GPe cells (index i) receive synapse from cells i and i - 1
        aux = random.sample(range(n_neurons), stn_gpe)
        connList = [[(x - c) % n_stn, x % n_gpe] for x in aux for c in [1, 0]]
        weight = [random.uniform(0, 0.002) * 0.43e-3 for k in range(len(connList))]
        for ch in range(self.n_channels):
            self.netParams.connParams['STN->GPe2_%d' % ch] = {
                'preConds': {'pop': 'STN_%d' % ch}, 'postConds': {'pop': 'GPe_%d' % ch},  # STN-> GPe
                'connList': connList,  # NMDA
                'weight': weight,  # synaptic weight (conductance)
                'delay': 2,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Insge,nmda'}  # target synaptic mechanism

        # GPe-< GPe connections
        n_neurons = self.netParams.popParams['GPe_0']['numCells']
        connList = [[(idx + ncn) % n_neurons, idx] for ncn in range(1, gpe_gpe + 1, 2)
                    for idx in range(n_neurons)] + \
                   [[idx, (idx + ncn) % n_neurons] for ncn in range(2, gpe_gpe + 1, 2)
                    for idx in range(n_neurons)]
        # connList = [[2,1],[3,2],[4,3],[5,4],[6,5],[7,6],[8,7],[9,8],[0,9],[1,0],
        #            [8,0],[9,1],[0,2],[1,3],[2,4],[3,5],[4,6],[5,7],[6,8],[7,9]]
        weight = [(0.25 + 0.75 * self.pd) * random.uniform(0, 1) * 0.3e-3 \
                  for k in range(len(connList))]
        for ch in range(self.n_channels):
            self.netParams.connParams['GPe->GPe_%d' % ch] = {
                'preConds': {'pop': 'GPe_%d' % ch}, 'postConds': {'pop': 'GPe_%d' % ch},  # GPe-< GPe
                'connList': connList,
                'weight': weight,  # synaptic weight (conductance)
                'delay': 1,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Igege'}  # target synaptic mechanism

        # StrD2>GPe connections
        n_strd2 = self.netParams.popParams['StrD2_0']['numCells']
        n_gpe = self.netParams.popParams['GPe_0']['numCells']
        for ch in range(self.n_channels):
            self.netParams.connParams['StrD2->GPe_%d' % ch] = {
                'preConds': {'pop': 'StrD2_%d' % ch}, 'postConds': {'pop': 'GPe_%d' % ch},  # StrD2-> GPe
                'connList': [[j, i] for i in range(n_gpe)
                             for j in range(n_strd2)],
                'weight': 0.15e-3,  # synaptic weight (conductance)
                'delay': 5,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Istrgpe'}  # target synaptic mechanism

    def gpiConnRules(self,
                     stn_gpi=5,
                     gpe_gpi=2,
                     **args):
        # STN-> GPi connections
        # Five aleatory GPi cells (index i) receive synapse from cells i and i - 1
        n_stn = self.netParams.popParams['STN_0']['numCells']
        n_gpi = self.netParams.popParams['GPi_0']['numCells']
        n_neurons = max(n_stn, n_gpi)
        aux = random.sample(range(n_neurons), stn_gpi)

        # PSTH
        self.gsngi = np.zeros(10)
        for k in range(0, 10):
            if (k == aux[0] or k == aux[1] or k == aux[2] or k == aux[3] or k == aux[4]):
                self.gsngi[k] = 1
            else:
                self.gsngi[k] = 0

        connList = [[(x - c) % n_stn, x % n_gpi] for x in aux for c in [1, 0]]
        for ch in range(self.n_channels):
            self.netParams.connParams['STN->GPi_%d' % ch] = {
                'preConds': {'pop': 'STN_%d' % ch}, 'postConds': {'pop': 'GPi_%d' % ch},
                'connList': connList,
                'weight': 0.0645e-3,  # synaptic weight (conductance)
                'delay': 1.5,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Isngi'}  # target synaptic mechanism

        # GPe-< GPi connections
        n_gpe = self.netParams.popParams['GPe_0']['numCells']
        n_gpi = self.netParams.popParams['GPi_0']['numCells']
        n_neurons = max(n_gpe, n_gpi)
        for ch in range(self.n_channels):
            self.netParams.connParams['GPe->GPi_%d' % ch] = {
                'preConds': {'pop': 'GPe_%d' % ch}, 'postConds': {'pop': 'GPi_%d' % ch},
                'connList':
                    [[idx % n_gpe, (idx + ncn) % n_gpi] for ncn in range(2, gpe_gpi + 1, 2)
                     for idx in range(n_neurons)] + \
                    [[(idx + ncn) % n_gpe, idx % n_gpi] for ncn in range(1, gpe_gpi + 1, 2)
                     for idx in range(n_neurons)],
                # [ [ idx, (idx+2) % n_neurons ] for idx in range( n_neurons ) ] + \
                # [ [ (idx+1) % n_neurons, idx ] for idx in range( n_neurons ) ],
                'weight': 0.15e-3,  # synaptic weight (conductance)
                'delay': 3,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Igegi'}  # target synaptic mechanism

        # StrD1>GPi connections
        n_strd1 = self.netParams.popParams['StrD1_0']['numCells']
        n_gpi = self.netParams.popParams['GPi_0']['numCells']
        for ch in range(self.n_channels):
            self.netParams.connParams['StrD1->GPe_%d' % ch] = {
                'preConds': {'pop': 'StrD1_%d' % ch}, 'postConds': {'pop': 'GPi_%d' % ch},  # StrD1-> GPi
                'connList': [[j, i] for i in range(n_gpi)
                             for j in range(n_strd1)],
                'weight': 0.15e-3,  # synaptic weight (conductance)
                'delay': 4,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Istrgpi'}  # target synaptic mechanism

    def stnConnRules(self, **args):
        # GPe-> STN connections
        n_gpe = self.netParams.popParams['GPe_0']['numCells']
        n_stn = self.netParams.popParams['STN_0']['numCells']
        n_neurons = max(n_gpe, n_stn)
        for ch in range(self.n_channels):
            self.netParams.connParams['GPe->STN_%d' % ch] = {
                'preConds': {'pop': 'GPe_%d' % ch}, 'postConds': {'pop': 'STN_%d' % ch},  # GPe-< STN
                'connList': [[(i + c) % n_gpe, i % n_stn] for c in [1, 0] for i in range(n_neurons)],
                'weight': 0.15e-3,  # synaptic weight (conductance)
                'delay': 4,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Igesn'}  # target synaptic mechanism

        # CTX-> STN connections
        n_ctxrs = self.netParams.popParams['CTX_RS_0']['numCells']
        n_stn = self.netParams.popParams['STN_0']['numCells']
        n_neurons = max(n_ctxrs, n_stn)
        connList = [[(i + c) % n_ctxrs, i % n_stn] for c in [1, 0] for i in range(n_neurons)]
        weight = [random.uniform(0, 0.3) * 0.43e-3 for k in range(len(connList))]
        for ch in range(self.n_channels):
            self.netParams.connParams['CTX->STN_%d' % ch] = {
                'preConds': {'pop': 'CTX_RS_%d' % ch}, 'postConds': {'pop': 'STN_%d' % ch},  # CTX-> STN
                'connList': connList,
                'weight': weight,  # synaptic weight (conductance)
                'delay': 5.9,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Icosn,ampa'}  # target synaptic mechanism
        # CTX-> STN2
        connList = [[(i + c) % n_ctxrs, i % n_stn] for c in [1, 0] for i in range(n_neurons)]
        weight = [random.uniform(0, 0.003) * 0.43e-3 for k in range(len(connList))]
        for ch in range(self.n_channels):
            self.netParams.connParams['CTX->STN2_%d' % ch] = {
                'preConds': {'pop': 'CTX_RS_%d' % ch}, 'postConds': {'pop': 'STN_%d' % ch},  # CTX-> STN
                'connList': connList,
                'weight': weight,  # synaptic weight (conductance)
                'delay': 5.9,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Icosn,nmda'}  # target synaptic mechanism

    def strConnRules(self,
                     strd2_strd2=4,
                     strd1_strd1=3,
                     gsynmod=1,
                     **args):
        # StrD2-< StrD2 connections
        # Each StrD2 cell receive synapse from 4 aleatory StrD2 cell (except from itself)
        n_neurons = self.netParams.popParams['StrD2_0']['numCells']
        connList = [[x, i] for i in range(n_neurons)
                    for x in random.sample([k for k in range(n_neurons) if k != i],
                                           strd2_strd2)]
        for ch in range(self.n_channels):
            self.netParams.connParams['StrD2->StrD2_%d' % ch] = {
                'preConds': {'pop': 'StrD2_%d' % ch}, 'postConds': {'pop': 'StrD2_%d' % ch},  # StrD2-< StrD2
                'connList': connList,
                'weight': 0.1 / 4 * 0.5e-3,  # synaptic weight (conductance) -> mudar essa maluquisse
                'delay': 0,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Igabaindr'}  # target synaptic mechanism

        # StrD1-< StrD1 connections
        # Each StrD1 cell receive synapse from 3 aleatory StrD1 cell (except from itself)
        n_neurons = self.netParams.popParams['StrD1_0']['numCells']
        connList = [[x, i] for i in range(n_neurons)
                    for x in random.sample([k for k in range(n_neurons) if k != i],
                                           strd1_strd1)]
        for ch in range(self.n_channels):
            self.netParams.connParams['StrD1->StrD1_%d' % ch] = {
                'preConds': {'pop': 'StrD1_%d' % ch}, 'postConds': {'pop': 'StrD1_%d' % ch},  # StrD1-< StrD1
                'connList': connList,
                'weight': 0.1 / 3 * 0.5e-3,  # synaptic weight (conductance) -> mudar aqui tb
                'delay': 0,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Igabadr'}  # target synaptic mechanism

        # RS-> StrD1 connections
        n_ctxrs = self.netParams.popParams['CTX_RS_0']['numCells']
        n_strd1 = self.netParams.popParams['StrD1_0']['numCells']
        n_neurons = max(n_ctxrs, n_strd1)
        for ch in range(self.n_channels):
            self.netParams.connParams['RS->StrD1_%d' % ch] = {
                'preConds': {'pop': 'CTX_RS_%d' % ch}, 'postConds': {'pop': 'StrD1_%d' % ch},  # RS-> StrD1
                'connList': [[i % n_ctxrs, i % n_strd1] for i in range(n_neurons)],
                'weight': (0.07 - 0.044 * self.pd) * 0.43e-3 * gsynmod,  # synaptic weight (conductance)
                'delay': 5.1,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Icostr'}  # target synaptic mechanism

        # RS-> StrD2 connections
        n_ctxrs = self.netParams.popParams['CTX_RS_0']['numCells']
        n_strd2 = self.netParams.popParams['StrD2_0']['numCells']
        n_neurons = max(n_ctxrs, n_strd2)
        for ch in range(self.n_channels):
            self.netParams.connParams['RS->StrD2_%d' % ch] = {
                'preConds': {'pop': 'CTX_RS_%d' % ch}, 'postConds': {'pop': 'StrD2_%d' % ch},  # RS-> StrD2
                'connList': [[i % n_ctxrs, i % n_strd2] for i in range(n_neurons)],
                'weight': 0.07 * 0.43e-3 * gsynmod,  # synaptic weight (conductance)
                'delay': 5.1,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Icostr'}  # target synaptic mechanism

    def ctxConnRules(self,
                     rs_fsi=4,
                     fsi_rs=4,
                     **args):
        # RS -> FSI connections
        # Each FSI cell receive synapse from 4 aleatory RS cells
        n_rs = self.netParams.popParams['CTX_RS_0']['numCells']
        n_fsi = self.netParams.popParams['CTX_FSI_0']['numCells']
        connList = [[x, i] for i in range(n_fsi)
                    for x in random.sample([k for k in range(n_rs) if k != i],
                                           rs_fsi)]
        for ch in range(self.n_channels):
            self.netParams.connParams['ctx_rs->ctx_fsi_%d' % ch] = {
                'preConds': {'pop': 'CTX_RS_%d' % ch}, 'postConds': {'pop': 'CTX_FSI_%d' % ch},  # ctx_rs -> ctx_fsi
                'connList': connList,
                'weight': 0.043e-3,  # synaptic weight (conductance)
                'delay': 1,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Iei'}  # target synaptic mechanism

        # FSI -> RS connections
        # Each RS cell receive synapse from 4 aleatory FSI cells
        connList = [[x, i] for i in range(n_rs)
                    for x in random.sample([k for k in range(n_fsi) if k != i],
                                           fsi_rs)]
        for ch in range(self.n_channels):
            self.netParams.connParams['ctx_fsi->ctx_rs_%d' % ch] = {
                'preConds': {'pop': 'CTX_FSI_%d' % ch}, 'postConds': {'pop': 'CTX_RS_%d' % ch},  # ctx_fsi -< ctx_rs
                'connList': connList,
                'weight': 0.083e-3,  # synaptic weight (conductance)
                'delay': 1,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Iie'}  # target synaptic mechanism

        # Th -> RS connections
        n_th = self.netParams.popParams['TH_0']['numCells']
        n_ctxrs = self.netParams.popParams['CTX_RS_0']['numCells']
        n_neurons = max(n_th, n_ctxrs)
        for ch in range(self.n_channels):
            self.netParams.connParams['th->ctx_rs_%d' % ch] = {
                'preConds': {'pop': 'TH_%d' % ch}, 'postConds': {'pop': 'CTX_RS_%d' % ch},  # th -> ctx_rs
                'connList': [[i % n_th, i % n_ctxrs] for i in range(n_neurons)],
                'weight': 0.0645e-3,  # synaptic weight (conductance)
                'delay': 5,  # transmission delay (ms)
                'loc': 1,  # location of synapse
                'synMech': 'Ithco'}  # target synaptic mechanism

    def buildStimParams(self,
                        amp_th=1.2e-3, amp_gpe=3e-3,
                        amp_gpi=3e-3, amp_stn=0,
                        amp_fs=0, amp_rs=0,
                        amp_dstr=0, amp_istr=0):
        bin_fs = 0;
        bin_rs = 0;
        bin_gpe = 0;
        bin_gpi = 0;
        bin_stn = 0;
        bin_dstr = 0;
        bin_istr = 0;
        bin_th = 0;

        for ch in range(self.n_channels):
            # FS receve a constante 3 density current or 1 during cortical stimulation
            self.netParams.stimSourceParams['Input_FS_%d' % ch] = {'type': 'IClamp',
                                                                   'delay': 0,
                                                                   'dur': self.t_sim,
                                                                   'amp': bin_fs * -1}
            self.netParams.stimTargetParams['Input_FS->FS_%d' % ch] = {'source': 'Input_FS_%d' % ch,
                                                                       'conds': {'pop': 'CTX_FSI_%d' % ch},
                                                                       'sec': 'soma',
                                                                       'loc': 0}

            # RS receve a constante 3 density current or 1 during cortical stimulation
            self.netParams.stimSourceParams['Input_RS_%d' % ch] = {'type': 'IClamp',
                                                                   'delay': 0,
                                                                   'dur': self.t_sim,
                                                                   'amp': bin_rs * -1 + amp_rs}
            self.netParams.stimTargetParams['Input_RS->RS_%d' % ch] = {'source': 'Input_RS_%d' % ch,
                                                                       'conds': {'pop': 'CTX_RS_%d' % ch},
                                                                       'sec': 'soma',
                                                                       'loc': 0}

            # GPe receve a constante 3 density current or 1 during cortical stimulation
            self.netParams.stimSourceParams['Input_GPe_%d' % ch] = {'type': 'IClamp',
                                                                    'delay': 0,
                                                                    'dur': self.t_sim,
                                                                    'amp': bin_gpe * -1 + amp_gpe}
            self.netParams.stimTargetParams['Input_GPe->GPe_%d' % ch] = {'source': 'Input_GPe_%d' % ch,
                                                                         'conds': {'pop': 'GPe_%d' % ch},
                                                                         'sec': 'soma',
                                                                         'loc': 0}

            # GPi receve a constante 3 density current
            self.netParams.stimSourceParams['Input_GPi_%d' % ch] = {'type': 'IClamp',
                                                                    'delay': 0, 'dur': self.t_sim,
                                                                    'amp': bin_gpi * -1 + amp_gpi}
            self.netParams.stimTargetParams['Input_GPi->GPi_%d' % ch] = {'source': 'Input_GPi_%d' % ch,
                                                                         'conds': {'pop': 'GPi_%d' % ch},
                                                                         'sec': 'soma',
                                                                         'loc': 0}

            # STN receve a constante 3 density current or 1 during cortical stimulation
            self.netParams.stimSourceParams['Input_STN_%d' % ch] = {'type': 'IClamp',
                                                                    'delay': 0,
                                                                    'dur': self.t_sim,
                                                                    'amp': bin_stn * -1 + amp_stn}
            self.netParams.stimTargetParams['Input_STN->STN_%d' % ch] = {'source': 'Input_STN_%d' % ch,
                                                                         'conds': {'pop': 'STN_%d' % ch},
                                                                         'sec': 'soma',
                                                                         'loc': 0}

            # dStr receve a constante 3 density current
            self.netParams.stimSourceParams['Input_StrD1_%d' % ch] = {'type': 'IClamp',
                                                                      'delay': 0,
                                                                      'dur': self.t_sim,
                                                                      'amp': bin_dstr * -1 + amp_dstr}
            self.netParams.stimTargetParams['Input_StrD1->StrD1_%d' % ch] = {'source': 'Input_StrD1_%d' % ch,
                                                                             'conds': {'pop': 'StrD1_%d' % ch},
                                                                             'sec': 'soma',
                                                                             'loc': 0}

            # iStr receve a constante 3 density current
            self.netParams.stimSourceParams['Input_StrD2_%d' % ch] = {'type': 'IClamp',
                                                                      'delay': 0, 'dur': self.t_sim,
                                                                      'amp': bin_istr * -1 + amp_istr}
            self.netParams.stimTargetParams['Input_StrD2->StrD2_%d' % ch] = {'source': 'Input_StrD2_%d' % ch,
                                                                             'conds': {'pop': 'StrD2_%d' % ch},
                                                                             'sec': 'soma',
                                                                             'loc': 0}

            # Thalamus receve a constante 1.2 density current
            self.netParams.stimSourceParams['Input_th_%d' % ch] = {'type': 'IClamp',
                                                                   'delay': 0,
                                                                   'dur': self.t_sim,
                                                                   'amp': bin_th * -1 + amp_th}
            self.netParams.stimTargetParams['Input_th->TH_%d' % ch] = {'source': 'Input_th_%d' % ch,
                                                                       'conds': {'pop': 'TH_%d' % ch},
                                                                       'sec': 'soma',
                                                                       'loc': 0}

    def extractLFP_SP(self):
        lfp = sim.allSimData['LFP']
        # [ f, t ]
        lfp = np.transpose(lfp, [1, 0])

        # calculate LFP using Welch method
        lfp_f, lfp_dimensions = signal.welch(lfp[0], 50, nperseg=1024, detrend=False)
        lfp_fft = np.zeros((len(self.electrodesPos) // self.nelec, lfp_dimensions.shape[0]))
        for i in range(0, lfp.shape[0], self.nelec):
            reg_fft = list()
            for j in range(self.nelec):
                reg_fft.append(signal.welch(lfp[i + j], 50, nperseg=1024, detrend=False))
            lfp_f, lfp_fft[i // self.nelec, :] = np.mean(reg_fft, axis=0)
        return lfp_f, lfp_fft

    def extractLFP_raw(self):
        lfp = sim.allSimData['LFP']
        # [ f, t ]
        lfp = np.transpose(lfp, [1, 0])
        return lfp

    def extractSpikes(self):
        spikes = self.Spikes
        spk_dict = dict()
        n_strd1 = self.netParams.popParams['StrD1_0']['numCells']
        n_strd2 = self.netParams.popParams['StrD2_0']['numCells']
        n_th = self.netParams.popParams['TH_0']['numCells']
        n_gpi = self.netParams.popParams['GPi_0']['numCells']
        n_gpe = self.netParams.popParams['GPe_0']['numCells']
        n_cor_rs = self.netParams.popParams['CTX_RS_0']['numCells']
        n_cor_fsi = self.netParams.popParams['CTX_FSI_0']['numCells']
        n_stn = self.netParams.popParams['STN_0']['numCells']

        c_strd1 = n_strd1
        c_strd2 = c_strd1 + n_strd2
        c_th = c_strd2 + n_th
        c_gpi = c_th + n_gpi
        c_gpe = c_gpi + n_gpe
        c_cor_rs = c_gpe + n_cor_rs
        c_cor_fsi = c_cor_rs + n_cor_fsi
        c_stn = c_cor_fsi + n_stn

        for ch in range(self.n_channels):
            spk_dict['dStr_APs_%d' % ch] = [spikes() for k in range(n_strd1)]
            spk_dict['iStr_APs_%d' % ch] = [spikes() for k in range(n_strd2)]
            spk_dict['TH_APs_%d' % ch] = [spikes() for k in range(n_th)]
            spk_dict['GPi_APs_%d' % ch] = [spikes() for k in range(n_gpi)]
            spk_dict['GPe_APs_%d' % ch] = [spikes() for k in range(n_gpe)]
            spk_dict['Cor_RS_APs_%d' % ch] = [spikes() for k in range(n_cor_rs)]
            spk_dict['Cor_FSI_APs_%d' % ch] = [spikes() for k in range(n_cor_fsi)]
            spk_dict['STN_APs_%d' % ch] = [spikes() for k in range(n_stn)]

        for ch in range(self.n_channels):
            for i in range(len(sim.allSimData['spkt'])):
                strd1_a = ch * c_stn
                strd2_a = ch * c_stn + c_strd1
                th_a = ch * c_stn + c_strd2
                gpi_a = ch * c_stn + c_th
                gpe_a = ch * c_stn + c_gpi
                rs_a = ch * c_stn + c_gpe
                fsi_a = ch * c_stn + c_cor_rs
                stn_a = ch * c_stn + c_cor_fsi
                strd1_b = strd1_a + n_strd1
                strd2_b = strd2_a + n_strd2
                th_b = th_a + n_th
                gpi_b = gpi_a + n_gpi
                gpe_b = gpe_a + n_gpe
                rs_b = rs_a + n_cor_rs
                fsi_b = fsi_a + n_cor_fsi
                stn_b = stn_a + n_stn

                if (sim.allSimData['spkid'][i] >= strd1_a and sim.allSimData['spkid'][i] < strd1_b):
                    spk_dict['dStr_APs_%d' % ch][int(sim.allSimData['spkid'][i] - strd1_a)].times += [
                        sim.allSimData['spkt'][i]]

                elif (sim.allSimData['spkid'][i] >= strd2_a and sim.allSimData['spkid'][i] < strd2_b):
                    spk_dict['iStr_APs_%d' % ch][int(sim.allSimData['spkid'][i] - strd2_a)].times += [
                        sim.allSimData['spkt'][i]]

                elif (sim.allSimData['spkid'][i] >= th_a and sim.allSimData['spkid'][i] < th_b):
                    spk_dict['TH_APs_%d' % ch][int(sim.allSimData['spkid'][i] - th_a)].times += [
                        sim.allSimData['spkt'][i]]

                elif (sim.allSimData['spkid'][i] >= gpi_a and sim.allSimData['spkid'][i] < gpi_b):
                    spk_dict['GPi_APs_%d' % ch][int(sim.allSimData['spkid'][i] - gpi_a)].times += [
                        sim.allSimData['spkt'][i]]

                elif (sim.allSimData['spkid'][i] >= gpe_a and sim.allSimData['spkid'][i] < gpe_b):
                    spk_dict['GPe_APs_%d' % ch][int(sim.allSimData['spkid'][i] - gpe_a)].times += [
                        sim.allSimData['spkt'][i]]

                elif (sim.allSimData['spkid'][i] >= rs_a and sim.allSimData['spkid'][i] < rs_b):
                    spk_dict['Cor_RS_APs_%d' % ch][int(sim.allSimData['spkid'][i] - rs_a)].times += [
                        sim.allSimData['spkt'][i]]

                elif (sim.allSimData['spkid'][i] >= fsi_a and sim.allSimData['spkid'][i] < fsi_b):
                    spk_dict['Cor_FSI_APs_%d' % ch][int(sim.allSimData['spkid'][i] - fsi_a)].times += [
                        sim.allSimData['spkt'][i]]

                elif (sim.allSimData['spkid'][i] >= stn_a and sim.allSimData['spkid'][i] < stn_b):
                    spk_dict['STN_APs_%d' % ch][int(sim.allSimData['spkid'][i] - stn_a)].times += [
                        sim.allSimData['spkt'][i]]

        return spk_dict

    def extractMFR(self):
        mfr = [sim.allSimData.popRates['StrD1_0'],
               sim.allSimData.popRates['StrD2_0'],
               sim.allSimData.popRates['TH_0'],
               sim.allSimData.popRates['GPi_0'],
               sim.allSimData.popRates['GPe_0'],
               sim.allSimData.popRates['CTX_RS_0'],
               sim.allSimData.popRates['CTX_FSI_0'],
               sim.allSimData.popRates['STN_0']]
        for i in range(0, 8):
            mfr[i] = round(mfr[i], 2)
        return mfr

    def get_gsngi(self):
        return self.gsngi

    def buildSimConfig(self, dt=0.1, lfp=False, recordStep=1, seeds=None):
        # Simulation parameters
        simConfig = specs.SimConfig()
        simConfig.duration = self.t_sim  # Duration of the simulation, in ms
        simConfig.dt = dt  # Internal integration timestep to use
        simConfig.verbose = False  # Show detailed messages
        simConfig.printPopAvgRates = True
        if seeds is not None:
            simConfig.seeds = seeds

        # Recording
        print('recordStep', recordStep)
        simConfig.recordStep = recordStep  # Step size in ms to save data (eg. V traces, LFP, etc)
        simConfig.recordCells = ['allCells']
        simConfig.recordSpikesGids = True
        # lfp and plot
        if lfp:
            simConfig.recordLFP = self.electrodesPos
            simConfig.saveLFPCells = True
            # simConfig.analysis['plotRaster'] = True
            # simConfig.analysis['plotLFP'] = {'electrodes': ['all'],
            #                                  'includeAxon': False,
            #                                  'timeRange': [0, 2000],
            #                                  'plots': ['timeSeries', 'locations', 'PSD'],
            #                                  'plots': ['locations'],
            #                                  'showFig': True}
        return simConfig

    def simulate(self, dt=0.1, lfp=False, seeds=None):

        simConfig = self.buildSimConfig(dt=dt, lfp=lfp, seeds=seeds)
        sim.initialize(  # create network object and set cfg and net params
            simConfig=simConfig,  # pass simulation config and network params as arguments
            netParams=self.netParams)
        sim.net.createPops()  # instantiate network populations
        sim.net.createCells()  # instantiate network cells based on defined populations
        sim.net.connectCells()  # create connections between cells based on params
        sim.net.addStims()  # add stimulation
        sim.setupRecording()  # setup variables to record for each cell (spikes, V traces, etc)

        sim.runSimWithIntervalFunc(self.stim_interval, self.IntervalFunc)
        for i in range(0, len(self.overall_interval_mfr)):
            print(self.overall_interval_mfr[i])
        # self.plotBetaPower()
        # self.plotPDCoeff()

        print(self.PDcoeff_table)
        sim.gatherData()  # gather spiking data and cell info from each node
        sim.saveData()
        # self.c.close()

        #return self.f, self.PSDstudy

if __name__ == '__main__':
    network = Structures(has_pd=True, n_channels=1, seed=i)
    PDcoeff = network.simulate()