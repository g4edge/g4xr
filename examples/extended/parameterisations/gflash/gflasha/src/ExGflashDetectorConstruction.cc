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
//
/// \file ExGflashDetectorConstruction.cc
/// \brief Implementation of the ExGflashDetectorConstruction class
//

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

// User Classes
#include "ExGflashDetectorConstruction.hh"

#include "ExGflashHomoShowerTuning.hh"
#include "ExGflashMessenger.hh"
#include "ExGflashSensitiveDetector.hh"

// G4 Classes
#include "G4AutoDelete.hh"
#include "G4Box.hh"
#include "G4Colour.hh"
#include "G4LogicalVolume.hh"
#include "G4Material.hh"
#include "G4NistManager.hh"
#include "G4PVPlacement.hh"
#include "G4RunManager.hh"
#include "G4SDManager.hh"
#include "G4SystemOfUnits.hh"
#include "G4ThreeVector.hh"
#include "G4VPhysicalVolume.hh"
#include "G4VisAttributes.hh"
#include "globals.hh"

// fast simulation
#include "GFlashHitMaker.hh"
#include "GFlashHomoShowerParameterisation.hh"
#include "GFlashParticleBounds.hh"
#include "GFlashShowerModel.hh"

#include "G4FastSimulationManager.hh"

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

const G4int kMaxBin = 500;

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

ExGflashDetectorConstruction::ExGflashDetectorConstruction()
{
  if (fVerbose > 3) G4cout << "ExGflashDetectorConstruction::Detector constructor" << G4endl;
  fGflashMessenger = new ExGflashMessenger(this);

  // Crystall
  fCrystalWidth = 3 * cm;
  fCrystalLength = 140 * cm;

  DefineMaterials();
  SetMaterial("G4_PbWO4");
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

ExGflashDetectorConstruction::~ExGflashDetectorConstruction()
{
  delete fGflashMessenger;
  delete fFastShowerModel;
  delete fParameterisation;
  delete fParticleBounds;
  delete fHitMaker;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void ExGflashDetectorConstruction::DefineMaterials()
{
  if (fVerbose > 3) G4cout << "Defining the materials" << G4endl;
  // Get nist material manager
  G4NistManager* nistManager = G4NistManager::Instance();
  // Build materials
  fHallMat = nistManager->FindOrBuildMaterial("G4_AIR");
  fDetMat = nistManager->FindOrBuildMaterial("G4_PbWO4");
  nistManager->FindOrBuildMaterial("G4_CESIUM_IODIDE");
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

G4VPhysicalVolume* ExGflashDetectorConstruction::Construct()
{
  //--------- Definitions of Solids, Logical Volumes, Physical Volumes ---------

  //------------------------------
  // Calorimeter segments
  //------------------------------
  // Simplified `CMS-like` PbWO4 crystal calorimeter

  // Calorimeter
  G4double calo_xside = (fCrystalWidth * fNbOfCrystals);
  G4double calo_yside = (fCrystalWidth * fNbOfCrystals);
  G4double calo_zside = fCrystalLength;

  // The Experimental Hall
  G4double experimentalHall_x = calo_xside * 4;
  G4double experimentalHall_y = calo_yside * 4;
  G4double experimentalHall_z = calo_zside * 4;

  G4VSolid* experimentalHall_box = new G4Box("expHall_box",  // World Volume
                                             experimentalHall_x,  // x size
                                             experimentalHall_y,  // y size
                                             experimentalHall_z);  // z size

  auto experimentalHall_log = new G4LogicalVolume(experimentalHall_box, fHallMat, "expHall_log",
                                                  nullptr,  // opt: fieldManager
                                                  nullptr,  // opt: SensitiveDetector
                                                  nullptr);  // opt: UserLimits
  G4VPhysicalVolume* experimentalHall_phys =
    new G4PVPlacement(nullptr,
                      G4ThreeVector(),  // at (0,0,0)
                      "expHall", experimentalHall_log, nullptr, false, 0);

  auto calo_box = new G4Box("Calorimeter",  // its name
                            calo_xside / 2.,  // size
                            calo_yside / 2., calo_zside / 2.);
  auto calo_log = new G4LogicalVolume(calo_box,  // its solid
                                      fHallMat,  // its material
                                      "calo_log",  // its name
                                      nullptr,  // opt: fieldManager
                                      nullptr,  // opt: SensitiveDetector
                                      nullptr);  // opt: UserLimit

  G4double xpos = 0.0;
  G4double ypos = 0.0;
  G4double zpos = calo_zside / 2.;  // face @ z= 0.0

  new G4PVPlacement(nullptr, G4ThreeVector(xpos, ypos, zpos), calo_log, "calorimeter",
                    experimentalHall_log, false, 1);

  // Crystals
  G4VSolid* crystal_box = new G4Box("Crystal",  // its name
                                    fCrystalWidth / 2, fCrystalWidth / 2, fCrystalLength / 2);
  // size
  fCrystal_log = new G4LogicalVolume(crystal_box,  // its solid
                                     fDetMat,  // its material
                                     "Crystal_log");  // its name

  for (G4int i = 0; i < fNbOfCrystals; i++) {
    for (G4int j = 0; j < fNbOfCrystals; j++) {
      G4int n = i * 10 + j;
      G4ThreeVector crystalPos((i * fCrystalWidth) - (calo_xside - fCrystalWidth) / 2.,
                               (j * fCrystalWidth) - (calo_yside - fCrystalWidth) / 2., 0);
      new G4PVPlacement(nullptr,  // no rotation
                        crystalPos,  // translation
                        fCrystal_log,
                        "crystal",  // its name
                        calo_log, false, n);
    }
  }
  G4cout << "There are " << fNbOfCrystals << " crystals per row in the calorimeter, so in total "
         << fNbOfCrystals * fNbOfCrystals << " crystals" << G4endl;
  G4cout << "They have width of  " << fCrystalWidth / cm << "  cm and a length of  "
         << fCrystalLength / cm << " cm." << G4endl;
  G4cout << fDetMat << G4endl;
  G4cout << "Total Calorimeter size " << calo_xside / cm << " cm x " << calo_yside / cm << " cm x "
         << calo_zside / cm << " cm" << G4endl;

  experimentalHall_log->SetVisAttributes(G4VisAttributes::GetInvisible());
  auto caloVisAtt = new G4VisAttributes(G4Colour(1.0, 1.0, 1.0));
  auto crystalVisAtt = new G4VisAttributes(G4Colour(1.0, 1.0, 0.0));
  calo_log->SetVisAttributes(caloVisAtt);
  fCrystal_log->SetVisAttributes(crystalVisAtt);

  // define the fParameterisation region
  //  G4cout << "\n ---> DetectorConstruction Region Definition" << G4endl;
  fRegion = new G4Region("crystals");
  calo_log->SetRegion(fRegion);
  fRegion->AddRootLogicalVolume(calo_log);

  return experimentalHall_phys;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void ExGflashDetectorConstruction::ConstructSDandField()
{
  // -- fast simulation models:
  // **********************************************
  // * Initializing shower modell
  // ***********************************************
  if (fParameterisation != nullptr) {
    fParameterisation->SetMaterial(fDetMat);
    if (fVerbose > 3) G4cout << "Info " << __func__ << " Param Mat " << fDetMat << G4endl;
    fParameterisation->PrintMaterial(fDetMat);
  }
  else {
    // -- sensitive detectors:

    G4SDManager* SDman = G4SDManager::GetSDMpointer();

    auto SD = new ExGflashSensitiveDetector("Calorimeter", this);

    SDman->AddNewDetector(SD);
    if (fCrystal_log != nullptr) {
      fCrystal_log->SetSensitiveDetector(SD);
    }

    if (fVerbose > 3) G4cout << "\n--> Creating shower parameterization models" << G4endl;
    fFastShowerModel = new GFlashShowerModel("fFastShowerModel", fRegion);
    fParameterisation =
      new GFlashHomoShowerParameterisation(fDetMat, new ExGflashHomoShowerTuning());
    fFastShowerModel->SetParameterisation(*fParameterisation);
    // Energy Cuts to kill particles:
    fParticleBounds = new GFlashParticleBounds();
    fFastShowerModel->SetParticleBounds(*fParticleBounds);
    // Makes the EnergieSpots
    fHitMaker = new GFlashHitMaker();
    fFastShowerModel->SetHitMaker(*fHitMaker);
    if (fVerbose > 3) G4cout << "end shower parameterization." << G4endl;
  }
  // **********************************************
  // Get Rad Len and R moliere from parameterisation
  fSDRadLen = fParameterisation->GetX0();
  fSDRm = fParameterisation->GetRm();
  if (fVerbose > 2) {
    G4cout << "Info " << __func__ << "Total Calorimeter size" << G4endl;
    auto calo_xyside = fCrystalWidth * fNbOfCrystals;
    G4cout << "Info Z " << __func__ << " " << fCrystalLength / cm << " cm "
           << fCrystalLength / fSDRadLen << " RadLen " << G4endl;
    G4cout << "Info XY " << __func__ << " " << calo_xyside / cm << " cm " << calo_xyside / fSDRm
           << " Rm " << G4endl;
  }
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void ExGflashDetectorConstruction::SetLBining(G4ThreeVector Value)
{
  fNLtot = (G4int)Value(0);
  if (fNLtot > kMaxBin) {
    G4cout << "\n ---> warning from SetLBining: " << fNLtot << " truncated to " << kMaxBin
           << G4endl;
    fNLtot = kMaxBin;
  }
  fDLradl = Value(1);
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void ExGflashDetectorConstruction::SetRBining(G4ThreeVector Value)
{
  fNRtot = (G4int)Value(0);
  if (fNRtot > kMaxBin) {
    G4cout << "\n ---> warning from SetRBining: " << fNRtot << " truncated to " << kMaxBin
           << G4endl;
    fNRtot = kMaxBin;
  }
  fDRradl = Value(1);
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void ExGflashDetectorConstruction::SetMaterial(G4String mat)
{
  // search the material by its name
  G4Material* pttoMaterial = G4NistManager::Instance()->FindOrBuildMaterial(mat);

  if (pttoMaterial != nullptr && fDetMat != pttoMaterial) {
    fDetMat = pttoMaterial;
    if (fCrystal_log != nullptr) {
      fCrystal_log->SetMaterial(fDetMat);
    }
    G4RunManager::GetRunManager()->PhysicsHasBeenModified();
  }
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
