package org.firstinspires.ftc.teamcode;

public class AutoOperation {
    public enum OpCode {
        OP_STOP,
        OP_DRIVE_TRAIN_RESET_ENCODER,
        OP_DRIVE_TRAIN_RESET_HEADING,
        OP_DRIVE_TRAIN_SHIFT_GEAR,
        OP_DRIVE_TRAIN_FORWARD,
        OP_DRIVE_TRAIN_BACKWARD,
        OP_DRIVE_TRAIN_TURN_LEFT,
        OP_DRIVE_TRAIN_TURN_RIGHT,
        OP_DRIVE_TRAIN_SHIFT_LEFT,
        OP_DRIVE_TRAIN_SHIFT_RIGHT
    };

    private OpCode opcode_;
    private double operand_;

    public AutoOperation(OpCode opcode,
                         double operand) {
        opcode_=opcode; operand_=operand;
    }

    public OpCode opcode() { return opcode_; }
    public double operand() { return operand_; }
}
