import re


op_codes = {
    'CALL': '001100',
    'LW': '010010',
    'NOT': '000000',
    'XOR': '000000',
    'ADD': '000000',
    'BNE': '001011',
    'SUBI': '100000',
    'BLE': '001010',
    'RET': '001100',
    'BEQ': '001000',
    'BGE': '001010',
    'SW': '011010',
    'JMP': '001100',
    'ADDI': '100000'
}

op2_codes = {
    'NOT': '101100',
    'XOR': '100110',
    'ADD': '100000'
}

reg_vals = {
    'S0': '110000',
    'Zero': '000000',
    'A0': '010000',
    'T0': '100000',
    'T1': '100001',
    'A1': '010001'
}


def get_insts():
    insts = []
    lines = []

    with open('Sorter3.a32', 'r') as f:
        for line in f.readlines():
            inst = line.strip().split()

            if len(inst) > 0:
                inst = inst[0].strip(';')

                if inst and not re.match('^;.*$', inst) and not re.match('^.*:$', inst) and not re.match('^\..*$', inst):
                    insts.append(inst)
                    lines.append(line.strip())

    return insts, lines


def write_first6(insts):
    with open('step1.txt', 'w') as f:
        f.write('\n'.join([op_codes[inst] for inst in insts]))


def finish_alur(insts, lines):
    alurs = []

    for i in range(len(lines)):
        inst = insts[i]
        line = lines[i]

        if op_codes[inst] == '000000' and inst != 'NOT':
            regs = line.split('\t')[2].split(',')
            regs = [reg_vals[reg] for reg in regs]
            alurs.append('000000' + regs[2] + regs[0] + regs[1] + '00' + op2_codes[inst])
        else:
            alurs.append(op_codes[inst])


    with open('step2.txt', 'w') as f:
        f.write('\n'.join(alurs))


def step3_to_hex():
    hex_vals = []

    with open('step3.txt', 'r') as f:
        for line in f.readlines():
            line = line.strip()
            hex_vals.append("%08X" % int(line, 2))

    with open('step4.txt', 'w') as f:
        f.write('\n'.join(hex_vals))


def main():
    step3_to_hex()


if __name__ == '__main__':
    main()
