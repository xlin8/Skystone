package org.firstinspires.ftc.teamcode;

public class AutoOperation {
    int opcode_;
    double operand_;

    public AutoOperation(int opcode,
                         double operand) {
        opcode_=opcode; operand_=operand;
    }

    public int opcode() { return opcode_; }
    public double operand() { return operand_; }
}
