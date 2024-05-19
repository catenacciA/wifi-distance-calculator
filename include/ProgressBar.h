#ifndef PROGRESSBAR_H
#define PROGRESSBAR_H

class IProgressBar {
public:
    virtual void display(int current, int total) = 0;
    virtual ~IProgressBar() = default;
};

class ProgressBar : public IProgressBar {
public:
    void display(int current, int total) override;
};

#endif // PROGRESSBAR_H
