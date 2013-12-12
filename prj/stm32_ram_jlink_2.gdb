monitor reg r13 = (0x20000000)
monitor reg pc = (0x20000004)

break ResetHandler
break main
continue
