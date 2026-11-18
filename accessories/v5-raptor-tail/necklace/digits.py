"""Three rounded cartoon fingers per paw; dimensions are shoulder-local millimetres."""
def digits(sign):
    for lane in (-1,0,1):
        middle=lane==0
        yield [((30.0,sign*(4+2.25*lane),-13.7),1.65),
               ((34.9 if middle else 33.7,sign*(4+4.0*lane),-13.0+(.7 if middle else 0)),1.42),
               ((38.7 if middle else 37.0,sign*(4+5.15*lane),-16.35 if middle else -15.9),1.12)]
