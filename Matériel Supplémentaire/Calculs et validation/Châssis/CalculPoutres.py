import numpy as np
import matplotlib.pyplot as plt

class PoutrePorteFauxBi:
    def __init__(self, L, E, I):
        self.L = L
        self.E = E
        self.I = I
        self.p = np.linspace(0, self.L, 1000)
        self.shear = np.zeros(len(self.p))
        self.bending = np.zeros(len(self.p))
        self.deflection = np.zeros(len(self.p))
        
    def addForce(self,F,x):
        a = x
        b = self.L - a
        Ra = F*(3*a+b)*b**2/self.L**3
        Rb = F*(3*b+a)*a**2/self.L**3
        Ma = -F*a*b**2/self.L**2
        Mb = -F*a**2*b/self.L**2
        Mf = 2*F*a**2*b**2/self.L**3
        deltaf = F*a**3*b**3/(3*self.L**3*self.E*self.I)
        
        V = []
        M = []
        delta = []
        
        for i in self.p:
            if i < a:
                V.append(Ra)
            elif i >= a:
                V.append(-Rb)
        
        for i in self.p:
            if i < a:
                slope = (Mf - Ma)/a
                M.append(Ma + slope*i)
                
            if i >= a:
                slope = (Mb - Mf)/b
                M0 = Mf - slope*a
                M.append(M0 + slope*i)
                
        for i in self.p:
            if i < a:
                slope = deltaf/a
                delta.append(slope*i)
                
            if i >= a:
                slope = -deltaf/b
                delta0 = deltaf - slope*a
                delta.append(delta0 + slope*i)
                
        self.shear += V
        self.bending += M
        self.deflection += delta
        
    def plotMVDiagram(self):
        fig,axs = plt.subplots(3)
        fig.suptitle('Diagramme de leffort tranchant et du moment fléchissant')
        
        axs[0].plot(self.p,self.shear, 'r')
        axs[0].fill_between(self.p,self.shear, color = '#ffb3b3') 
        axs[0].axhline(0, color='k', linewidth = 0.5)
        axs[0].set_xlim(0, p.p[-1])
        axs[0].set(ylabel='Effort tranchant (N)')
        
        axs[1].plot(self.p,self.bending,'b')
        axs[1].fill_between(self.p,self.bending, color = '#b3b3ff')
        axs[1].axhline(0, color='k', linewidth = 0.5)
        axs[1].set_xlim(0, p.p[-1])
        axs[1].set(ylabel='Moment fléchissant (Nm)')
        
        axs[2].plot(self.p,self.deflection,'g')
        axs[2].fill_between(self.p,self.deflection, color = '#a9de95')
        axs[2].axhline(0, color='k', linewidth = 0.5)
        axs[2].set(xlabel='Position dans la poutre (m)', ylabel='Deflection (Nm)')
        axs[2].set_xlim(0, p.p[-1])
        
        print('Effort tranchant maximal : ', max(abs(self.shear)), ' Newtons')
        print('Moment fléchissant maximal : ', max(abs(self.bending)), 'Newton-mètres')
        
        
class PoutrePorteFauxUni:
    def __init__(self, L, E, I):
        self.L = L
        self.E = E
        self.I = I
        self.p = np.linspace(0, self.L, 1000)
        self.shear = np.zeros(len(self.p))
        self.bending = np.zeros(len(self.p))
        self.deflection = np.zeros(len(self.p))
        
    def addForce(self,F,x):
        a = x
        b = self.L - a
        Ra = F*(3*a+b)*b**2/self.L**3
        Rb = F*(3*b+a)*a**2/self.L**3
        Ma = -F*a*b**2/self.L**2
        Mb = -F*a**2*b/self.L**2
        Mf = 2*F*a**2*b**2/self.L**3
        deltaf = F*a**3*b**3/(3*self.L**3*self.E*self.I)
        
        V = []
        M = []
        delta = []
        
        for i in self.p:
            if i < a:
                V.append(Ra)
            elif i >= a:
                V.append(-Rb)
        
        for i in self.p:
            if i < a:
                slope = (Mf - Ma)/a
                M.append(Ma + slope*i)
                
            if i >= a:
                slope = (Mb - Mf)/b
                M0 = Mf - slope*a
                M.append(M0 + slope*i)
                
        for i in self.p:
            if i < a:
                slope = deltaf/a
                delta.append(slope*i)
                
            if i >= a:
                slope = -deltaf/b
                delta0 = deltaf - slope*a
                delta.append(delta0 + slope*i)
                
        self.shear += V
        self.bending += M
        self.deflection += delta
        
    def plotMVDiagram(self):
        fig,axs = plt.subplots(3)
        fig.suptitle('Diagramme de leffort tranchant et du moment fléchissant')
        
        axs[0].plot(self.p,self.shear, 'r')
        axs[0].fill_between(self.p,self.shear, color = '#ffb3b3') 
        axs[0].axhline(0, color='k', linewidth = 0.5)
        axs[0].set_xlim(0, p.p[-1])
        axs[0].set(xlabel='Déformation approximatif dans la poutre (m)', ylabel='Effort tranchant (N)')
        
        axs[1].plot(self.p,self.bending,'b')
        axs[1].fill_between(self.p,self.bending, color = '#b3b3ff')
        axs[1].axhline(0, color='k', linewidth = 0.5)
        axs[1].set(ylabel='Moment fléchissant (Nm)')
        axs[1].set_xlim(0, p.p[-1])
        
        
        axs[2].plot(self.p,self.deflection,'g')
        axs[2].fill_between(self.p,self.deflection, color = '#a9de95')
        axs[2].axhline(0, color='k', linewidth = 0.5)
        axs[2].set(xlabel='Déformation approximatif dans la poutre (m)', ylabel='Deflection (Nm)')
        axs[2].set_xlim(0, p.p[-1])
        
        print('Effort tranchant maximal : ', max(abs(self.shear)), ' Newtons')
        print('Moment fléchissant maximal : ', max(abs(self.bending)), 'Newton-mètres')
        

        
p = PoutrePorteFauxBi(392,69000,28300)
p.addForce(-25, 108)
p.addForce(-160,392/2)
p.addForce(-25, 284)
p.plotMVDiagram()
plt.show()