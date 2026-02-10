## Transforming Policy-Car Swerving for Mitigating Stop-and-Go Traffic Waves: A Practical Jam-Absorption Driving Strategy


**[Background]** Stop-and-go waves, as a major form of freeway traffic congestion, cause severe and long-lasting adverse effects, including reduced traffic efficiency, increased driving risks, and higher vehicle emissions.

<img src="fig/Figure_1.png" width="70%">

**[JAD Strategy]** Amongst the highway traffic management strategies, jam-absorption driving (JAD), in which a dedicated vehicle performs "slow-in" and "fast-out" maneuvers before being captured by a stop-and-go wave, has been proposed as a potential method for preventing the propagation of such waves. 

**[Motivation]** However, most existing JAD strategies remain impractical mainly due to the lack of discussion regarding implementation vehicles and operational conditions.

**[Contribution]** 
- Inspired by real-world observations of police-car swerving behavior, this paper first introduces a *Single-Vehicle Two-Detector Jam-Absorption Driving (SVDD-JAD)* problem, and then proposes a practical JAD strategy that transforms such behavior into a maneuver capable of suppressing the propagation of an isolated stop-and-go wave.  
- Five key parameters that significantly affect the proposed strategy, namely, *JAD speed*, *inflow traffic speed*, *wave width*, *wave speed*, and *in-wave speed*, are identified and systematically analyzed. 
- Using a SUMO-based simulation as an illustrative example, we further demonstrate how these parameters can be measured in practice with two stationary roadside traffic detectors. 
The results show that the proposed JAD strategy successfully suppresses the propagation of a stop-and-go wave, without triggering a secondary wave.

**[Significance]** This paper is expected to take a significant step toward making JAD practical, **advancing it from a theoretical concept to a feasible and implementable strategy**.


<img src="fig/Figure_2.png" width="40%">

**[Video] Police-Car Swerving on Freeway, https://www.youtube.com/watch?v=_IvmWaSDorg**

**[Video] Police-Car Swerving on Freeway, https://www.youtube.com/watch?v=_IvmWaSDorg**

**[Video] Police-Car Swerving on Freeway, https://www.youtube.com/watch?v=_IvmWaSDorg**

**[Video] Police-Car Swerving on Freeway, https://www.youtube.com/watch?v=_IvmWaSDorg**


 

 

  

## Code

**[Baseline]**

*b_1_simu_base.py*

*b_2_simu_base_plot_tx.py*

<img src="fig/Figure_11.png" width="40%">

**[Stability and JAD Speed]**

*c_1_simu_stability.py*

*c_2_simu_stability_plot_tx.py*

<img src="fig/Figure_12.png" width="40%">

**[Simulation]**

*d_1_simu_jad.py*

*d_4_simu_jad_plot_detector.py*

<img src="fig/Figure_13.png" width="40%">

*d_5_simu_jad_plot_headway.py*

<img src="fig/Figure_14.png" width="40%">

*d_2_simu_jad_plot_tx.py*

<img src="fig/Figure_15.png" width="40%">

*d_3_simu_jad_plot_tx_failed.py*

<img src="fig/Figure_16.png" width="40%">



## Preprint
https://arxiv.org/abs/**


## Citation

If you find this work useful, please consider citing our paper:

```bibtex
@article{hezhengbing2026,
  title={Transforming Policy-Car Swerving for Mitigating Stop-and-Go Traffic Waves: A Practice-Oriented Jam-Absorption Driving Strategy},
  author={He, Zhengbing},
  journal={arXiv preprint arXiv:},
  year={2026}
}
