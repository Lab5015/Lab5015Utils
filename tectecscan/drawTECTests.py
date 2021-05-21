#! /usr/bin/python
import sys
import ROOT
import time
from datetime import datetime
from optparse import OptionParser

parser = OptionParser()

parser.add_option("--nArrays", dest="nArrays", default=1)
parser.add_option("--run", dest="run", default="")
(options, args) = parser.parse_args()

inFile = 'run'+str(options.run)+'.txt'
outFileName = 'run'+str(options.run)+'.root'
outFile = ROOT.TFile.Open(outFileName, 'RECREATE')
Tmin = 0.
Tmax = 60.

labels = []
colors = {}
labels.append("I");                 colors["I"] = ROOT.kBlack
labels.append("V");                 colors["V"] = ROOT.kBlack
labels.append("LYSO");              colors["LYSO"] = ROOT.kTeal
labels.append("Warm TEC side");     colors["Warm TEC side"] = ROOT.kOrange
labels.append("Cold plate");        colors["Cold plate"] = ROOT.kBlue
labels.append("Hot TEC side");      colors["Hot TEC side"] = ROOT.kRed
labels.append("SiPMs");             colors["SiPMs"] = ROOT.kViolet
labels.append("Humidity");          colors["Humidity"] = ROOT.kAzure
labels.append("Air");               colors["Air"] = ROOT.kGreen+2
labels.append("TEC power");         colors["TEC power"] = ROOT.kBlack
labels.append("TEC power scaled");  colors["TEC power scaled"] = ROOT.kBlack

graphs = {}
values = {}

timestamp_first = 0
timestamp_last = 0



for label in labels:
    graphs[label] = ROOT.TGraph()

with open(str(inFile), 'r') as fin:
    for line in fin.readlines():
        readings = line.strip().split()
        
        date = datetime.strptime(readings[0]+" "+readings[1], '%Y-%m-%d %H:%M:%S')
        timestamp = time.mktime(date.timetuple())
        if timestamp_first == 0:
            timestamp_first = timestamp
        if timestamp > timestamp_last:
            timestamp_last = timestamp
        
        for it in range(2,len(readings)):
            values[labels[it-2]] = float(readings[it])
            graph = graphs[labels[it-2]]
            graph.SetPoint(graph.GetN(),(timestamp-timestamp_first)/60.,float(readings[it]))
    
        graph = graphs["TEC power"]
        graph.SetPoint(graph.GetN(),(timestamp-timestamp_first)/60.,values["I"]*values["V"]*1000./float(options.nArrays))


c1 = ROOT.TCanvas('','',1300,600)
c1.SetGridx()
c1.SetGridy()
hPad1 = ROOT.gPad.DrawFrame(0.-0.05*(timestamp_last-timestamp_first)/60.,Tmin,1.05*(timestamp_last-timestamp_first)/60.,Tmax)
hPad1.SetTitle(";time elapsed [min];T [#circ C]")
hPad1.Draw()

graphs["SiPMs"].SetLineColor(colors["SiPMs"])
graphs["SiPMs"].SetLineWidth(1)
graphs["SiPMs"].Draw("Lsame")

graphs["Warm TEC side"].SetLineColor(colors["Warm TEC side"])
graphs["Warm TEC side"].SetLineWidth(1)
graphs["Warm TEC side"].Draw("Lsame")

graphs["Hot TEC side"].SetLineColor(colors["Hot TEC side"])
graphs["Hot TEC side"].SetLineWidth(1)
graphs["Hot TEC side"].Draw("Lsame")

graphs["Cold plate"].SetLineColor(colors["Cold plate"])
graphs["Cold plate"].SetLineWidth(1)
graphs["Cold plate"].Draw("Lsame")

graphs["LYSO"].SetLineColor(colors["LYSO"])
graphs["LYSO"].SetLineWidth(1)
graphs["LYSO"].Draw("Lsame")

graphs["Air"].SetLineColor(colors["Air"])
graphs["Air"].SetLineWidth(1)
graphs["Air"].Draw("Lsame")

ymin = 0.
ymax = 1500.
axis = ROOT.TGaxis(ROOT.gPad.GetUxmax(),ROOT.gPad.GetUymin(),ROOT.gPad.GetUxmax(),ROOT.gPad.GetUymax(),ymin,ymax,510,"+L")
axis.SetTitleColor(ROOT.kBlack);
axis.SetLabelColor(ROOT.kBlack);
axis.SetLabelFont(42);
axis.SetLabelSize(0.045)
axis.SetTitle("TEC power [mW]")
axis.SetTitleFont(42)
axis.SetTitleSize(0.06)
axis.SetTitleOffset(0.8)
axis.Draw();

graphs["TEC power scaled"] = ROOT.TGraph()
for point in range(graphs["TEC power"].GetN()):
    graphs["TEC power scaled"].SetPoint(point,graphs["TEC power"].GetPointX(point),(graphs["TEC power"].GetPointY(point)-ymin)*Tmax/(ymax-ymin))
graphs["TEC power scaled"].SetLineColor(colors["TEC power"])
graphs["TEC power scaled"].Draw("Lsame")



graph_TECPower_vs_deltaT = ROOT.TGraph()
graph_TECPower_vs_deltaTInit = ROOT.TGraph()

point_change = -1
val_change = 0.
tSiPMInit = 0.

for point in range(graphs["V"].GetN()):
    if point_change == -1:
        point_change = point
        val_change = graphs["V"].GetPointY(point)
    
    diff = graphs["V"].GetPointY(point) - val_change
    
    if abs(diff) > 0.001:
        point_change = point
        val_change = graphs["V"].GetPointY(point)        
        
        tmax = graphs["V"].GetPointX(point-1)
        tmin = graphs["V"].GetPointX(point-25)
        
        labels_to_fit = ["SiPMs", "Hot TEC side", "TEC power scaled", "TEC power", "Cold plate"]
        funcs = {}
        for label in labels_to_fit:
            funcs[label] = ROOT.TF1("func"+label,"[0]",tmin,tmax)
            graphs[label].Fit(funcs[label],"QNRS")
            funcs[label].SetLineColor(colors[label])
            funcs[label].SetLineWidth(3)
            if label != "TEC power":
                funcs[label].Draw("same")
        
        graph_TECPower_vs_deltaT.SetPoint(graph_TECPower_vs_deltaT.GetN(),funcs["SiPMs"].GetParameter(0)-funcs["Hot TEC side"].GetParameter(0),funcs["TEC power"].GetParameter(0))
        graph_TECPower_vs_deltaTInit.SetPoint(graph_TECPower_vs_deltaTInit.GetN(),funcs["SiPMs"].GetParameter(0) - funcs["Cold plate"].GetParameter(0),funcs["TEC power"].GetParameter(0))


c2 = ROOT.TCanvas('','',1300,600)
c2.SetGridx()
c2.SetGridy()
hPad2 = ROOT.gPad.DrawFrame(-25.,0.,30.,1600.)
hPad2.SetTitle(";#DeltaT [#circ C];TEC power [mW]")
hPad2.Draw()
graph_TECPower_vs_deltaT.SetMarkerStyle(20)
graph_TECPower_vs_deltaT.SetMarkerSize(1.3)
graph_TECPower_vs_deltaT.SetMarkerColor(ROOT.kRed)
graph_TECPower_vs_deltaT.SetLineColor(ROOT.kRed)
graph_TECPower_vs_deltaT.SetLineWidth(2)
graph_TECPower_vs_deltaT.Draw("PL,same")

graph_TECPower_vs_deltaTInit.SetMarkerStyle(24)
graph_TECPower_vs_deltaTInit.SetMarkerSize(1.3)
graph_TECPower_vs_deltaTInit.SetMarkerColor(ROOT.kRed)
graph_TECPower_vs_deltaTInit.SetLineColor(ROOT.kRed)
graph_TECPower_vs_deltaTInit.SetLineWidth(2)
graph_TECPower_vs_deltaTInit.SetLineStyle(7)
graph_TECPower_vs_deltaTInit.Draw("PL,same")

graph_arjan_LYSO = ROOT.TGraph("arjan_module_LYSO.txt")
graph_arjan_LYSO.SetLineColor(ROOT.kGreen+1)
graph_arjan_LYSO.SetLineWidth(2)
graph_arjan_LYSO.Draw("L,same")

graph_arjan_lab1 = ROOT.TGraph("arjan_lab1.txt")
graph_arjan_lab1.SetLineColor(ROOT.kViolet)
graph_arjan_lab1.SetLineWidth(2)
graph_arjan_lab1.Draw("L,same")

graph_arjan_lab2 = ROOT.TGraph("arjan_lab2.txt")
graph_arjan_lab2.SetLineColor(ROOT.kBlue)
graph_arjan_lab2.SetLineWidth(2)
graph_arjan_lab2.Draw("L,same")

leg = ROOT.TLegend(0.47, 0.89-5*0.04, 0.89, 0.89)
leg.AddEntry(graph_TECPower_vs_deltaT , 'our meas. (module w/ LYSO on cold plate wrt. hot plate)', 'PL')
leg.AddEntry(graph_TECPower_vs_deltaTInit , 'our meas. (module w/ LYSO on cold plate wrt. cold plate)', 'PL')
leg.AddEntry(graph_arjan_LYSO , 'Arjan\'s meas. (module w/ LYSO on cold plate)', 'PL')
leg.AddEntry(graph_arjan_lab1 , 'Arjan\'s meas. (SiPM array in lab. setup)', 'PL')
leg.AddEntry(graph_arjan_lab2 , 'Arjan\'s meas. (SiPM array in lab. setup)', 'PL')
leg.SetTextFont(42)
leg.SetTextSize(0.03)
leg.Draw('same')

outFile.cd()
graph_arjan_LYSO.Write("graph_arjan_LYSO")
graph_arjan_lab2.Write("graph_arjan_lab2")
graph_arjan_lab1.Write("graph_arjan_lab1")
graph_TECPower_vs_deltaTInit.Write("graph_TECPower_vs_deltaTInit")
graph_TECPower_vs_deltaT.Write("graph_TECPower_vs_deltaT")
graphs["TEC power scaled"].Write("graph_TEC_Power")
raw_input('ok?')
