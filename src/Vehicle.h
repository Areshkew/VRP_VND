#ifndef Vehicle_H
#define Vehicle_H

class Vehicle{
    private:
        int VehicleID, remainingCap;

    public:
        const static int speedKMH = 35;
        const static int maxCap = 3000;
        Vehicle();

        //Setter / Getters
        int getVehID();
        void setVehID(int newVehId);
        int getRemainingCap();
        void setRemainingCap(int newRemainingCap);
        int getMaxCap();
        void printVehData();
};

#endif