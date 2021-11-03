void imu_analysis(char* fileName){

    // Silence warnings during TTree::ReadFile
    Int_t currentIgnoreLevel = gErrorIgnoreLevel;
    gErrorIgnoreLevel = kError;
    //read the file and access the TTree
    TTree *t = new TTree("t", "Histogram");
    t->ReadFile(fileName, "Timestamp/D:Heading/D:tiltCompensatedHeading/D:kalmanX/D:kalmanY/D");
    // Restore previous settings
    gErrorIgnoreLevel = currentIgnoreLevel;
    t->Print();

    //define the variables to be read in the TTree
    double Timestamp;
    double kalmanX, kalmanY, Heading, tiltCompensatedHeading;


    //Set the proper addresses
    t->SetBranchAddress("Timestamp", &Timestamp);
    t->SetBranchAddress("Heading", &Heading);
    t->SetBranchAddress("tiltCompensatedHeading", &tiltCompensatedHeading);
    t->SetBranchAddress("kalmanX", &kalmanX);
    t->SetBranchAddress("kalmanY", &kalmanY);

    //Get the number of entries in the TTree
    int n = t->GetEntries();

    //Crates the Graph for the plot
    auto *graph_Heading = new TGraph();
    auto *graph_tiltCompensatedHeading = new TGraph();
    auto *graph_kalmanX = new TGraph();
    auto *graph_kalmanY = new TGraph();

    double first_timestamp = 0.;
    double time, time_save;


    //loop on events
    for(int ev=0; ev<=n; ++ev){
        //read the current event
        t->GetEntry(ev);
        if(ev==0) first_timestamp = Timestamp;
        if(ev%100000==0)   cout<< "Event #"<< ev << endl;

        time = (Timestamp-first_timestamp) / 3600.;
        if(time>=0){ time_save = time; }
        else {time = time_save +Timestamp/3600;}
        //fill the Graph
        graph_kalmanX->SetPoint(ev, time , kalmanX );
        graph_Heading->SetPoint(ev, time , Heading );
        graph_tiltCompensatedHeading->SetPoint(ev, time , tiltCompensatedHeading );
        graph_kalmanY->SetPoint(ev, time , kalmanY );

    }

    //Crates the frame for the plot
    //Bellurie for the plot

    auto canvas = new TCanvas("canvas","",1000,600);
    canvas->Divide(2,2);

    canvas->cd(1);
    graph_kalmanX->SetTitle("kalmanX");
    graph_kalmanX->SetMarkerStyle(8);
    graph_kalmanX->SetMarkerColor(kRed);
    graph_kalmanX->SetLineColor(kRed);
    graph_kalmanX->SetMarkerSize(0.5);
    graph_kalmanX->SetTitle("Angolo X (Pitch); Time[h]; Degrees");
    //graph_kalmanX->SetMaximum(1.);  graph_kalmanX->SetMinimum(0.);
    graph_kalmanX->Draw("AL");
    gPad->BuildLegend();
    gPad->SetGrid();

    canvas->cd(2);
    graph_kalmanY->SetTitle("kalmanY");
    graph_kalmanY->SetMarkerStyle(8);
    graph_kalmanY->SetMarkerColor(kRed);
    graph_kalmanY->SetLineColor(kRed);
    graph_kalmanY->SetMarkerSize(0.5);
    graph_kalmanY->SetTitle("Angolo Y (Roll); Time[h]; Degrees");
    //graph_kalmanY->SetMaximum(1.);  graph_kalmanY->SetMinimum(0.);
    graph_kalmanY->Draw("AL");
    gPad->BuildLegend();
    gPad->SetGrid();

    canvas->cd(3);
    graph_Heading->SetTitle("Heading");
    graph_Heading->SetMarkerStyle(8);
    graph_Heading->SetMarkerColor(kRed);
    graph_Heading->SetLineColor(kRed);
    graph_Heading->SetMarkerSize(0.5);
    graph_Heading->SetTitle("Angolo heading (Nord magnetico); Time[h]; Degrees");
    //graph_Heading->SetMaximum(90.); graph_Heading->SetMinimum(70);
    graph_Heading->Draw("AL");
    gPad->BuildLegend();
    gPad->SetGrid();


    canvas->cd(4);
    graph_tiltCompensatedHeading->SetTitle("tiltCompensatedHeading");
    graph_tiltCompensatedHeading->SetMarkerStyle(8);
    graph_tiltCompensatedHeading->SetMarkerColor(kRed);
    graph_tiltCompensatedHeading->SetLineColor(kRed);
    graph_tiltCompensatedHeading->SetMarkerSize(0.5);
    graph_tiltCompensatedHeading->SetTitle("Angolo heading (Nord magnetico) compensato; Time[h]; Degrees");
    //graph_tiltCompensatedHeading->SetMaximum(90.); graph_tiltCompensatedHeading->SetMinimum(70);
    graph_tiltCompensatedHeading->Draw("AL");
    gPad->SetGrid();
    gPad->BuildLegend();


}
