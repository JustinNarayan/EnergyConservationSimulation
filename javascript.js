/// Declare Input Variables
let blockMass,
   springConstant,
   compressionDistance,
   rampAngle,
   coefficientKineticFriction; // the μk value

/// Declare constants
const gravAcceleration = 9.8;

/// Initialize Simulation Variables
let time,
   sectionEndTimes = {
      spring: 0,
      surface: 0,
      ramp: 0,
      air: 0,
   };

let maximumSystemEnergy, energyLostToFriction;

let forceGravity, forceNormal;

let springPotentialEnergy,
   angularFrequency, // the ω value
   springPeriod,
   springTimeToEquilibrium;

let blockKineticEnergy,
   blockPotentialEnergy,
   blockPositionX,
   blockPositionY,
   blockVelocity,
   blockVelocityX,
   blockVelocityY,
   frictionalAcceleration, // dependent on μk and block weight
   blockAngle;

/// Listen for initial page load
window.addEventListener("load", updateInputs, false);

/// updateInputs()
/// ~ read inputs from user, display numerical values, and update simulation
function updateInputs() {
   /// Read Inputs
   blockMass = parseFloat(document.getElementById("block-mass").value);
   springConstant = parseFloat(
      document.getElementById("spring-constant").value
   );
   compressionDistance = parseFloat(
      document.getElementById("compression-distance").value
   );
   rampAngle = parseFloat(document.getElementById("ramp-angle").value);
   coefficientKineticFriction = parseFloat(
      document.getElementById("coefficient-kinetic-friction").value
   );

   /// Write inputs to displays
   document.getElementById("display-block-mass").innerHTML = `${blockMass} kg`;
   document.getElementById(
      "display-spring-constant"
   ).innerHTML = `${springConstant} N/m`;
   document.getElementById(
      "display-compression-distance"
   ).innerHTML = `${compressionDistance} m`;
   document.getElementById(
      "display-ramp-angle"
   ).innerHTML = `${rampAngle} &deg;`;
   document.getElementById(
      "display-coefficient-kinetic-friction"
   ).innerHTML = `${coefficientKineticFriction}`;

   time = parseFloat(document.getElementById("time").value);
   document.getElementById("display-time").innerHTML = `${time}`;

   /// Update temporary data readout
   computeValues();
}

/// computeValues();
/// ~ using user inputs, calculate all relevant values for the simulation
function computeValues() {
   /// Compute the relevant spring variables
   computeSpringValues(); //*should occur only once

   /// Compute the sectionEndTimes
   computeSectionEndTimes(); //*should occur only once

   /// Compute the position- and velocity- state of the block
   computeBlockValues();

   /// Compute the relevant energies
   computeEnergy();

   updateTempDataOutput();
}

/// computeSpringValues();
/// ~ calculate the spring period, angular frequnecy ω, and duration of contact with block
function computeSpringValues() {
   // Calculate the period of the spring, angular frequency ω, and the duration of time the block is in contact
   angularFrequency = Math.sqrt(springConstant / blockMass);
   springPeriod = (2 * Math.PI) / angularFrequency;
   springTimeToEquilibrium = springPeriod / 4; // how long the block is touching
}

/// computeSectionEndTimes();
/// ! calculate the end time (i.e. duration) of each section
function computeSectionEndTimes() {
   sectionEndTimes.spring = springTimeToEquilibrium;
   sectionEndTimes.surface = 1000; //TEMP
}

/// computeBlockValues();
/// ~calculate the position and velocity of the block based on its elapsed interactions with the spring, frictional surfaces, ramp, and air
function computeBlockValues() {
   /// Track the block's end-state after each time section
   let accumulated = {
      positionX: 0,
      positionY: 0,
      velocityX: 0,
      velocityY: 0,
   };

   /// Reset block's state
   blockPositionX = 0;
   blockPositionY = 0;
   blockVelocityX = 0;
   blockVelocityY = 0;

   // Calculate frictional acceleration opposite direction of motion
   frictionalAcceleration =
      coefficientKineticFriction * blockMass * gravAcceleration;

   // Generate all relevant kinematics equations
   let xPositionFromSpring = (elapsedTime) =>
      -1 * compressionDistance * Math.cos(angularFrequency * elapsedTime);
   let xVelocityFromSpring = (elapsedTime) =>
      compressionDistance *
      angularFrequency *
      Math.sin(angularFrequency * elapsedTime);

   /// Spring section
   if (time < sectionEndTimes.spring) {
      blockPositionX = xPositionFromSpring(time);
      blockPositionY = 0;

      blockVelocityX = xVelocityFromSpring(time);
      blockVelocityY = 0;
   } else {
      // Find end-state values after leaving spring
      accumulated.positionX += xPositionFromSpring(springTimeToEquilibrium);
      accumulated.velocityX += xVelocityFromSpring(springTimeToEquilibrium);
   }

   // Net Block State
   blockPositionX += accumulated.positionX;
   blockPositionY += accumulated.positionY;
   blockVelocityX += accumulated.velocityX;
   blockVelocityY += accumulated.velocityY;
   blockVelocity = Math.sqrt(
      Math.pow(blockVelocityX, 2) + Math.pow(blockVelocityY, 2)
   );
}

/// computeEnergy();
/// ~ calculate the kinetic, potential, and lost energy in the simulation
function computeEnergy() {
   /// Maximum System Energy
   maximumSystemEnergy =
      (1 / 2) * springConstant * Math.pow(compressionDistance, 2);

   /// Spring Potential Energy
   let currentSpringDisplacement = Math.min(blockPositionX, 0); // once the block leaves the spring, energy = 9
   springPotentialEnergy =
      (1 / 2) * springConstant * Math.pow(currentSpringDisplacement, 2);

   /// Block Kinetic Energy
   blockKineticEnergy = (1 / 2) * blockMass * Math.pow(blockVelocity, 2);

   /// Block Potential Energy
   blockPotentialEnergy = blockMass * gravAcceleration * blockPositionY;

   /// Energy Lost To Friction
   energyLostToFriction =
      maximumSystemEnergy -
      (springPotentialEnergy + blockKineticEnergy + blockPotentialEnergy);
}

/// updateTempDataOutput();
/// ~ calculate some data to check the simulation's physics engine
function updateTempDataOutput() {
   let page = document.getElementById("temp-data-output");

   /// Clear page then write information
   page.innerHTML = ``;
   page.innerHTML += `Maximum Energy: ${maximumSystemEnergy} <br>`;
   page.innerHTML += `Spring PE: ${springPotentialEnergy} <br>`;
   page.innerHTML += `Block KE: ${blockKineticEnergy} <br>`;
   page.innerHTML += `Block PE: ${blockPotentialEnergy} <br>`;
   page.innerHTML += `Energy Lost: ${energyLostToFriction} <br>`;
   page.innerHTML += `<br>`;
   page.innerHTML += `Spring Time To Equilibrium: ${sectionEndTimes.spring} <br>`;
   page.innerHTML += `Block X: ${blockPositionX} <br>`;
   page.innerHTML += `Block Velocity X: ${blockVelocityX} <br>`;
}
