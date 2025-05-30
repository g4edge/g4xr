//
// ********************************************************************
// * License and Disclaimer                                           *
// *                                                                  *
// * The  Geant4 software  is  copyright of the Copyright Holders  of *
// * the Geant4 Collaboration.  It is provided  under  the terms  and *
// * conditions of the Geant4 Software License,  included in the file *
// * LICENSE and available at  http://cern.ch/geant4/license .  These *
// * include a list of copyright holders.                             *
// *                                                                  *
// * Neither the authors of this software system, nor their employing *
// * institutes,nor the agencies providing financial support for this *
// * work  make  any representation or  warranty, express or implied, *
// * regarding  this  software system or assume any liability for its *
// * use.  Please see the license in the file  LICENSE  and URL above *
// * for the full disclaimer and the limitation of liability.         *
// *                                                                  *
// * This  code  implementation is the result of  the  scientific and *
// * technical work of the GEANT4 collaboration.                      *
// * By using,  copying,  modifying or  distributing the software (or *
// * any work based  on the software)  you  agree  to acknowledge its *
// * use  in  resulting  scientific  publications,  and indicate your *
// * acceptance of all terms of the Geant4 Software license.          *
// ********************************************************************
//
// Hadrontherapy advanced example for Geant4
// See more at: https://twiki.cern.ch/twiki/bin/view/Geant4/AdvancedExamplesHadrontherapy
//
// Using the builder concepts of Geant4 we assembled (and tested) two different
// Physics Lists that are particuilarly suited for Hadronterapy applications:
//
// 'HADRONTHERAPY_1' is more suited for protons only
// 'HADRONTHERAPY_2' is suggested for better precision with ions
//
// The Reference physics lists (already present in the Geant4 kernel) can
// be used as well. In this case the more suitable "Reference physics lists" are:
// "QBBC", "QGSP_BIC", "Shielding", "QGSP_BERT",
// "QGSP_BIC_AllHP" and "QGSP_BIC_HP"
//
// NOTE: to activate the "_HP" physics you have to set the G4PARTICLEHPDATA environment
// variable pointing to the external dataset named "G4TENDL".
//
// All the lists can be  activated inside any macro file using the command:
// /Physics/addPhysics
//
// Examples of usage are:
// /Physics/addPhysics HADRONTHERAPY_1 or /Physics/addPhysics QGSP_BIC_HP

#include "G4SystemOfUnits.hh"
#include "G4RunManager.hh"
#include "G4Region.hh"
#include "G4RegionStore.hh"
#include "HadrontherapyPhysicsList.hh"
#include "HadrontherapyPhysicsListMessenger.hh"
#include "HadrontherapyStepMax.hh"
#include "G4PhysListFactory.hh"
#include "G4VPhysicsConstructor.hh"
#include "G4HadronPhysicsQGSP_BIC_HP.hh"
#include "G4HadronPhysicsQGSP_BIC.hh"
#include "G4EmStandardPhysics_option4.hh"
#include "G4EmStandardPhysics.hh"
#include "G4EmExtraPhysics.hh"
#include "G4StoppingPhysics.hh"
#include "G4DecayPhysics.hh"
#include "G4HadronElasticPhysics.hh"
#include "G4HadronElasticPhysicsHP.hh"
#include "G4RadioactiveDecayPhysics.hh"
#include "G4IonBinaryCascadePhysics.hh"
#include "G4DecayPhysics.hh"
#include "G4NeutronTrackingCut.hh"
#include "G4LossTableManager.hh"
#include "G4UnitsTable.hh"
#include "G4ProcessManager.hh"
#include "G4IonFluctuations.hh"
#include "G4IonParametrisedLossModel.hh"
#include "G4EmParameters.hh"
#include "G4ParallelWorldPhysics.hh"
#include "G4EmLivermorePhysics.hh"
#include "G4AutoDelete.hh"
#include "G4HadronPhysicsQGSP_BIC_AllHP.hh"
#include "QGSP_BIC_HP.hh"
#include "QGSP_BIC.hh"
#include "G4HadronPhysicsQGSP_BERT.hh"
#include "G4HadronPhysicsQGSP_BERT_HP.hh"
#include "G4ParallelWorldPhysics.hh"
// Physics List
#include "QBBC.hh"
#include "QGSP_BIC.hh"
#include "Shielding.hh"
#include "QGSP_BERT.hh"
#include "QGSP_BIC_AllHP.hh"
#include "QGSP_BIC_HP.hh"



/////////////////////////////////////////////////////////////////////////////
HadrontherapyPhysicsList::HadrontherapyPhysicsList() : G4VModularPhysicsList()
{
    G4LossTableManager::Instance();
    defaultCutValue = 1.*mm;
    cutForGamma     = defaultCutValue;
    cutForElectron  = defaultCutValue;
    cutForPositron  = defaultCutValue;
    
    pMessenger = new HadrontherapyPhysicsListMessenger(this);
    SetVerboseLevel(1);
    decay_List = new G4DecayPhysics();
    // Elecromagnetic physics
    //
    emPhysicsList = new G4EmStandardPhysics_option4();
}

/////////////////////////////////////////////////////////////////////////////
HadrontherapyPhysicsList::~HadrontherapyPhysicsList()
{
    delete pMessenger;
    delete emPhysicsList;
    delete decay_List;
    //delete radioactiveDecay_List;
    hadronPhys.clear();
    for(size_t i=0; i<hadronPhys.size(); i++)
    {
        delete hadronPhys[i];
    }
}

/////////////////////////////////////////////////////////////////////////////
void HadrontherapyPhysicsList::ConstructParticle()
{
    decay_List -> ConstructParticle();
    
}

/////////////////////////////////////////////////////////////////////////////
void HadrontherapyPhysicsList::ConstructProcess()
{
    // Transportation
    //
    AddTransportation();
    
    decay_List -> ConstructProcess();
    emPhysicsList -> ConstructProcess();
    
    
    //em_config.AddModels();
    
    // Hadronic physics
    //
    for(size_t i=0; i < hadronPhys.size(); i++)
    {
        hadronPhys[i] -> ConstructProcess();
    }
    
    // step limitation (as a full process)
    //
    AddStepMax();
    
    //Parallel world sensitivity
    //
    G4ParallelWorldPhysics* pWorld = new G4ParallelWorldPhysics("DetectorROGeometry");
    pWorld->ConstructProcess();
    
    return;
}

/////////////////////////////////////////////////////////////////////////////
void HadrontherapyPhysicsList::AddPhysicsList(const G4String& name)
{
    if (verboseLevel>1) {
        G4cout << "PhysicsList::AddPhysicsList: <" << name << ">" << G4endl;
    }
    if (name == emName) return;
    
    ///////////////////////////////////
    //   ELECTROMAGNETIC MODELS
    ///////////////////////////////////
    if (name == "standard_opt4") {
        emName = name;
        delete emPhysicsList;
        hadronPhys.clear();
        emPhysicsList = new G4EmStandardPhysics_option4();
        G4RunManager::GetRunManager() -> PhysicsHasBeenModified();
        G4cout << "THE FOLLOWING ELECTROMAGNETIC PHYSICS LIST HAS BEEN ACTIVATED: G4EmStandardPhysics_option4" << G4endl;
        
        ////////////////////////////////////////
        //   ELECTROMAGNETIC + HADRONIC MODELS
        ////////////////////////////////////////
        
    }  else if (name == "HADRONTHERAPY_1") {
        
        AddPhysicsList("standard_opt4");
        hadronPhys.push_back( new G4DecayPhysics());
        hadronPhys.push_back( new G4RadioactiveDecayPhysics());
        hadronPhys.push_back( new G4IonBinaryCascadePhysics());
        hadronPhys.push_back( new G4EmExtraPhysics());
        hadronPhys.push_back( new G4HadronElasticPhysicsHP());
        hadronPhys.push_back( new G4StoppingPhysics());
        hadronPhys.push_back( new G4HadronPhysicsQGSP_BIC_HP());
        hadronPhys.push_back( new G4NeutronTrackingCut());
        
        G4cout << "HADRONTHERAPY_1 PHYSICS LIST has been activated" << G4endl;
    }
    
    else if (name == "HADRONTHERAPY_2") {
        
        AddPhysicsList("standard_opt4");
        hadronPhys.push_back( new G4DecayPhysics());
        hadronPhys.push_back( new G4RadioactiveDecayPhysics());
        hadronPhys.push_back( new G4IonBinaryCascadePhysics());
        hadronPhys.push_back( new G4EmExtraPhysics());
        hadronPhys.push_back( new G4HadronElasticPhysics());
        hadronPhys.push_back( new G4StoppingPhysics());
        hadronPhys.push_back( new G4HadronPhysicsQGSP_BIC_AllHP());
        hadronPhys.push_back( new G4NeutronTrackingCut());
        
        G4cout << "HADRONTHERAPY_2 PHYSICS LIST has been activated" << G4endl;   

    }
   
    else if (name == "QGSP_BIC"){
        auto physicsList = new QGSP_BIC;
        G4RunManager::GetRunManager() -> SetUserInitialization(physicsList);
        G4RunManager::GetRunManager() -> PhysicsHasBeenModified();
        physicsList -> RegisterPhysics(new G4ParallelWorldPhysics("DetectorROGeometry"));
    }
    
    else if (name == "QGSP_BERT"){
        auto physicsList = new QGSP_BERT;
        G4RunManager::GetRunManager() -> SetUserInitialization(physicsList);
        G4RunManager::GetRunManager() -> PhysicsHasBeenModified();
        physicsList -> RegisterPhysics(new G4ParallelWorldPhysics("DetectorROGeometry"));
    }
    
    else if (name == "QGSP_BIC_AllHP"){
        auto physicsList = new QGSP_BIC_AllHP;
        G4RunManager::GetRunManager() -> SetUserInitialization(physicsList);
        G4RunManager::GetRunManager() -> PhysicsHasBeenModified();
        physicsList -> RegisterPhysics(new G4ParallelWorldPhysics("DetectorROGeometry"));
    }
    
    else if (name == "QGSP_BIC_HP"){
        auto physicsList = new QGSP_BIC_HP;
        G4RunManager::GetRunManager() -> SetUserInitialization(physicsList);
        G4RunManager::GetRunManager() -> PhysicsHasBeenModified();
        physicsList -> RegisterPhysics(new G4ParallelWorldPhysics("DetectorROGeometry"));
    }
    
    else if (name == "Shielding"){
        auto physicsList = new Shielding;
        G4RunManager::GetRunManager() -> SetUserInitialization(physicsList);
        G4RunManager::GetRunManager() -> PhysicsHasBeenModified();
        physicsList -> RegisterPhysics(new G4ParallelWorldPhysics("DetectorROGeometry"));
    }
        
    else if (name == "QBBC"){
        auto physicsList = new QBBC;
        G4RunManager::GetRunManager() -> SetUserInitialization(physicsList);
        G4RunManager::GetRunManager() -> PhysicsHasBeenModified();
        physicsList -> RegisterPhysics(new G4ParallelWorldPhysics("DetectorROGeometry"));
    }
    
    else {
        G4cout << "PhysicsList::AddPhysicsList: <" << name << ">"
        << " is not defined"
        << G4endl;
    }
    
}

/////////////////////////////////////////////////////////////////////////////
void HadrontherapyPhysicsList::AddStepMax()
{
    // Step limitation seen as a process
    // This process must exist in all threads.
    //
    HadrontherapyStepMax* stepMaxProcess  = new HadrontherapyStepMax();
    
    
    auto particleIterator = GetParticleIterator();
    particleIterator->reset();
    while ((*particleIterator)()){
        G4ParticleDefinition* particle = particleIterator->value();
        G4ProcessManager* pmanager = particle->GetProcessManager();
        
        if (stepMaxProcess->IsApplicable(*particle) && pmanager)
        {
            pmanager ->AddDiscreteProcess(stepMaxProcess);
        }
    }
}
