//===----------------------------------------------------------------------===//
//  Declarations that describe the Simple register file 
//===----------------------------------------------------------------------===//

class SimpleReg<string n> : Register<n> {
  let Namespace = "Simple";
}

// Registers
//
def R0  : SimpleReg<"First">;
def R1  : SimpleReg<"Second">;
def R2  : SimpleReg<"Third">; 
def R3  : SimpleReg<"Fourth">;
def R4  : SimpleReg<"NO_IT_IS_TOO_MUCH_REGISTERS">;

// Register classes.
//
def GPR32 : RegisterClass<"Simple", [i32], 32,
  (add R0, R1, R2, R3, R4)>;
