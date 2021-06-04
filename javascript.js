/// Initialize Input Variables
let blockMass, springConstant, compressionDistance, rampAngle;

/// Listen for initial page load
window.addEventListener("load", updateInputs, false);

/// updateInputs()
/// ~ read inputs from user, display numerical values, and update simulation
function updateInputs() {
   /// Read Inputs
   blockMass = parseInt(document.getElementById("block-mass").value);
   springConstant = parseInt(document.getElementById("spring-constant").value);
   compressionDistance = parseInt(
      document.getElementById("compression-distance").value
   );
   rampAngle = parseInt(document.getElementById("ramp-angle").value);

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
}
