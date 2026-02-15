## Transforming Policy-Car Swerving for Mitigating Stop-and-Go Traffic Waves: A Practical Jam-Absorption Driving Strategy


**[Background]** 
Stop-and-go waves, as a major form of freeway traffic congestion, cause severe and long-lasting adverse effects, including reduced traffic efficiency, increased driving risks, and higher vehicle emissions.

**[JAD Strategy]** 
Amongst the highway traffic management strategies, jam-absorption driving (JAD), in which a dedicated vehicle performs "slow-in" and "fast-out" maneuvers before being captured by a stop-and-go wave, has been proposed as a potential method for preventing the propagation of such waves. 

**[Motivation]** 
However, most existing JAD strategies remain impractical mainly due to the lack of discussion regarding implementation vehicles and operational conditions.

**[Contribution]**
- Inspired by real-world observations of police-car swerving behavior, this paper first introduces a *Single-Vehicle Double-Detector Jam-Absorption Driving (SD-JAD)* problem, and then proposes a practical JAD strategy that transforms such behavior into a maneuver capable of suppressing the propagation of an isolated stop-and-go wave.  
- Five key parameters that significantly affect the proposed strategy, namely, *JAD speed*, *inflow traffic speed*, *wave width*, *wave speed*, and *in-wave speed*, are identified and systematically analyzed. 
- Using a SUMO-based simulation as an illustrative example, we further demonstrate how these parameters can be measured in practice with two stationary roadside traffic detectors. 
The results show that the proposed JAD strategy successfully suppresses the propagation of a stop-and-go wave, without triggering a secondary wave.

**[Significance]** 
This paper is expected to take a significant step toward making JAD practical, *advancing it from a theoretical concept to a feasible and implementable strategy*.

<br>

<br>


<img src="fig/swerving.png" width="100%">

*Police-car swerving at high speed on a freeway, California, USA. It shows that the police car was swerving at a high speed of about 70 km/h (44 mph) for roughly one minute. In the end, it caused the inflow to join a queue at a relatively low speed, implying a possible task aimed at slowing down traffic arrival.*

[Video] Police-Car Swerving on Freeway, https://www.youtube.com/watch?v=_IvmWaSDorg

[Video] Police-Car Swerving on Freeway, https://www.youtube.com/watch?v=_IvmWaSDorg



<br>


## Code

**[Baseline]**
 
    b_1_simu_base.py

    b_2_simu_base_plot_tx.py

<img src="fig/Figure_11.png" width="40%">

**[Stability and JAD Speed]**

    c_1_simu_stability.py

    c_2_simu_stability_plot_tx.py

<img src="fig/Figure_12.png" width="40%">

**[Simulation]**

    d_1_simu_jad.py

    d_4_simu_jad_plot_detector.py

<img src="fig/Figure_13.png" width="40%">

    d_5_simu_jad_plot_headway.py

<img src="fig/Figure_14.png" width="40%">

    d_2_simu_jad_plot_tx.py

<img src="fig/Figure_15.png" width="40%">

    d_3_simu_jad_plot_tx_failed.py

<img src="fig/Figure_16.png" width="40%">


<br>


## Preprint
[https://arxiv.org/abs/2602.10234](https://arxiv.org/abs/2602.10234)


<br>


## Citation

If you find this work useful, please consider citing our paper:

```
@article{he2026jad,
  title={Transforming Police-Car Swerving for Mitigating Stop-and-Go Traffic Waves: A Practice-Oriented Jam-Absorption Driving Strategy},
  author={He, Zhengbing},
  journal={arXiv preprint},
  year={2026},
  doi={10.48550/arXiv.2602.10234}
}
```

<br>


## Extended Reading

- *He, Z., Laval, J., Han, Y., Hegyi, A., Nishi, R., Wu, C.*  
  **A Review of Stop-and-Go Traffic Wave Suppression Strategies: Variable Speed Limit vs. Jam-Absorption Driving**  
  IEEE Transactions on Intelligent Transportation Systems, Accepted, 2026.  
  [Paper](https://arxiv.org/abs/2504.11372) | DOI: 10.1109/TITS.2026.3658644  

<img src="fig/JAD2017.png" width="50%">

- *He, Z., Zheng, L., Song, L., Zhu, N.*  
  **A Jam-Absorption Driving Strategy for Mitigating Traffic Oscillations**  
  IEEE Transactions on Intelligent Transportation Systems, 18(4): 802–813, 2017.  
  [Paper](https://www.researchgate.net/profile/Zhengbing-He/publication/305400445_A_Jam-Absorption_Driving_Strategy_for_Mitigating_Traffic_Oscillations/links/5b3d7e474585150d23fde2ee/A-Jam-Absorption-Driving-Strategy-for-Mitigating-Traffic-Oscillations.pdf) | DOI: 10.1109/TITS.2016.2587699

<img src="fig/JAD2026.png" width="50%">

<br>








