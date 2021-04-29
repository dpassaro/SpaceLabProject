#include "TCanvas.h"
#include "TFrame.h"
#include "TBenchmark.h"
#include "TString.h"
#include "TF1.h"
#include "TH1.h"
#include "TFile.h"
#include "TTreeReader.h"
#include "TTreeReaderValue.h"
#include "TTreeReaderArray.h"
#include "TROOT.h"
#include "TError.h"
#include "TInterpreter.h"
#include "TSystem.h"
#include "TPaveText.h"
#include "math.h"
#include "TThread.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <string>

double conv(double *, double *);
double michel_vuoto(double * ,double *);
double risoluzione(double *, double *);
double conv_0(double *, double *);
double __dist_GEANT__(double *, double *);

TF1 *dist_GEANT, *risol, *f0;
TH1F *dist_GEANT_;
TGraph *graph;


double __dist_GEANT__(double *en, double *par){
  int index = en[0]*200./1817.;
  return dist_GEANT_->GetBinContent(index);
}

double risoluzione(double *en, double *par){
  /*if(par[0]<650){ // energia in keV
    return TMath::Gaus(en[0], 0, par[0]*6/100);
  }*/
  return TMath::Gaus(en[0], 0, 29.9);
}

double conv_0(double *x, double *par){
  risol->SetParameter(0, par[0]);
  return dist_GEANT->Eval(x[0])*risol->Eval(par[0]-x[0]);
}

double conv(double *x, double *par){
  f0->SetParameter(0,x[0]);
  ROOT::Math::IntegratorOneDim ig(ROOT::Math::IntegrationOneDim::kADAPTIVE);
  ig.SetFunction(*f0);
  return ig.Integral(0,1817);
}


void Spettro_NaI(const char *input_file="file_prova_spettro_20_massa_0.root"){

  TChain *h = new TChain("t");
  h->Add(input_file);

  TCanvas *c1 = new TCanvas("Spettro energetico NaI","Spettro energetico NaI",1400,800);
  c1->Divide(3,2);  //c1->Divide(3,2); se con sorgente vera

  c1->cd(1);
  h->Draw("et[1]>>E_tot(200,0.,1817.)","pdg==22 || pdg==11 || pdg==-11");
  ((TH1F*) gPad->GetPrimitive("E_tot"))->SetTitle("Spettro energetico complessivo; Energia depositata [keV]");
  dist_GEANT_= (TH1F*) gPad->GetPrimitive("E_tot")->Clone("E_tot_new");
  dist_GEANT = new TF1("vuoto", __dist_GEANT__ , 0, 1817., 0);//selezionare qui la funzione da convolvere con la gaussiana
  risol = new TF1("risoluzione", risoluzione, 0 , 1817., 1);
  f0 = new TF1("conv_0", conv_0, 0, 1817., 1);
  TF1 *f_Conv0 = new TF1("Convoluzione E_tot", conv, 0, 1817., 0 );// dello spettro di Michel con gaussiana con sigma = 3MeV
  TH1F *E_tot_conv = new TH1F("Convoluzione","Convoluzione E_tot;Energia [keV]; Conteggi", 200,0,1817.);
  for(int i=0; i<100000; i++){ E_tot_conv->Fill(f_Conv0->GetRandom()); }
  c1->cd(4);
  E_tot_conv->Draw();
  //f_Conv0->Draw();f_Conv0->SetNpx(500);
//////////////////////////////////////////////////////////////////////////////
  c1->cd(2);
  h->Draw("et[1]>>E_gamma(200,0.,1817.)", "pdg==22");
  ((TH1F*) gPad->GetPrimitive("E_gamma"))->SetTitle("Spettro di tutti i fotoni; Energia depositata [keV]");
  dist_GEANT_= (TH1F*) gPad->GetPrimitive("E_gamma")->Clone("E_gamma_new");
  dist_GEANT = new TF1("vuoto", __dist_GEANT__ , 0, 1817., 0);//selezionare qui la funzione da convolvere con la gaussiana
  risol = new TF1("risoluzione", risoluzione, 0 , 1817., 1);
  f0 = new TF1("conv_0", conv_0, 0, 1817., 1);
  TF1 *f_Conv1 = new TF1("Convoluzione E_gamma", conv, 0, 1817., 0 );// dello spettro di Michel con gaussiana con sigma = 3MeV
  TH1F *E_gamma_conv = new TH1F("Convoluzione","Convoluzione E_gamma;Energia [keV]; Conteggi", 200,0,1817.);
  for(int i=0; i<100000; i++){ E_gamma_conv->Fill(f_Conv1->GetRandom()); }
  c1->cd(5);
  E_gamma_conv->Draw();
  //f_Conv1->Draw(); f_Conv1->SetNpx(500);

//////////////////////////////////////////////////////////////////////////////
  c1->cd(3);
  h->Draw("et[1]>>E_gamma_decay(200,0.,1817.)", "pdg==22 && pid==0 "); // (pid==2 || (trk-pid>=3 && pid>4) ) per sorgente vera
  ((TH1F*) gPad->GetPrimitive("E_gamma_decay"))->SetTitle("Spettro dei fotoni di decadimento; Energia depositata [keV]");
  dist_GEANT_= (TH1F*) gPad->GetPrimitive("E_gamma_decay")->Clone("E_gamma_decay_new");
  dist_GEANT = new TF1("vuoto", __dist_GEANT__ , 0, 1817., 0);//selezionare qui la funzione da convolvere con la gaussiana
  risol = new TF1("risoluzione", risoluzione, 0 , 1817., 1);
  f0 = new TF1("conv_0", conv_0, 0, 1817., 1);
  TF1 *f_Conv2 = new TF1("Convoluzione E_gamma_decay", conv, 0, 1817., 0 );// dello spettro di Michel con gaussiana con sigma = 3MeV
  TH1F *E_gamma_decay_conv = new TH1F("Convoluzione","Convoluzione E_gamma_decay;Energia [keV]; Conteggi", 200,0,1817.);
  for(int i=0; i<100000; i++){ E_gamma_decay_conv->Fill(f_Conv2->GetRandom()); }
  c1->cd(6);
  E_gamma_decay_conv->Draw();
  //f_Conv2->Draw(); f_Conv2->SetNpx(500);
}


void Frammentazione(const char *input_file="sciame.root"){

  TChain *h = new TChain("t");
  h->Add(input_file);

  TCanvas *c = new TCanvas("Produzione secondari","Produzione secondari",1200,600);
  c->Divide(1,1);
  c->cd(1);
  h->Draw("(z+24000000)/1000000>>hnew(100,0,42)" , "pro==4121 && pid==00");
  ((TH1F*) gPad->GetPrimitive("hnew"))->SetTitle("Distribuzione quota di frammentazione dei raggi cosmici primari ;Altitudine [km]");
  ((TH2F*) gPad->GetPrimitive("hnew"))->SetFillColor(kBlue); ((TH2F*) gPad->GetPrimitive("hnew"))->SetFillStyle(3002);
  ((TH2F*) gPad->GetPrimitive("hnew"))->SetLineWidth(2);
  /*
  c->cd(1);
  h->Draw("x:z*0.906 + y*0.423", "vlm==1 && pdg==22", "zcolor");//da aggiustare per l'angolo di osservazione
  ((TH2F*) gPad->GetPrimitive("htemp"))->SetTitle("Distribuzione angolare;z[mm];x[mm]");*/
  //TH1F *proiezione = ((TH2F*) gPad->GetPrimitive("htemp"))->ProjectionY();

  //c->cd(2);
  //proiezione->Draw();
}

void Particelle(const char *input_file="sciame.root"){
  TChain *h = new TChain("t");
  h->Add(input_file);

  TCanvas *c1 = new TCanvas("Particelle","Particelle",1200,400);
  c1->Divide(3,1);

  c1->cd(1);
  h->Draw("pdg>>htemp0(5000,-400, 400)","pdg<1000 && stp==0");
  ((TH1F*) gPad->GetPrimitive("htemp0"))->SetTitle("Particelle leggere");
  gPad->SetLogy();
  c1->cd(2);
  h->Draw("pdg>>htemp1(5000,2000,3400)", "pdg>1000 && pdg<80000000 && stp==0");
  ((TH1F*) gPad->GetPrimitive("htemp1"))->SetTitle("Adroni");
  gPad->SetLogy();
  c1->cd(3);
  h->Draw("(pdg-1000000000)/10000>>htemp2(5000)", "pdg>80000000 && stp==0");
  ((TH1F*) gPad->GetPrimitive("htemp2"))->SetTitle("Ioni; Z+0.001 #times A");
  gPad->SetLogy();

}
