import math
import numpy as np

dp=[1,1,1]
P=[0,0,0]
angle=0.5
current_angle=0
err=0
prev_err=0
summ_err=0
threshold=0.001

def summ(err):
    global summ_err
    summ_err=summ_err+err
    return summ_err

def diff(error):
    global prev_err

    return error-prev_err

    
def A(P):
    global err
    err = angle-current_angle
    return err
    
def curr_angle(P):
    global current_angle
    global prev_err
    global err
    angular_speed = P[0]*err + P[1]*summ(err) + P[2]*diff(err)/0.08
    current_angle=angular_speed*0.08
    #print current_angle
    prev_err=err

best_error=A(P)

#print best_error

while (sum(dp))>threshold or current_angle!=angle:
    for i in range(len(P)):

        P[i]=P[i]+dp[i]
        curr_angle(P)
        error=A(P)
        print error

        if error<best_error:
            best_error=error
            dp[i]=dp[i]*1.1
            print 'H'
        else:
            P[i]=P[i]-2*dp[i]
            curr_angle(P)
            error=A(P)
            print 'A'

            if error<best_error:
                best_error=error
                dp[i]=dp[i]*1.05
                print 'R'

            else:
                P[i]=P[i]+dp[i]
                dp[i] *=0.95
                print 'I'
        
    
print P    
print dp
print current_angle
