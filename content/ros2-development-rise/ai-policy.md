+++
title = "AI & LLM policy"
weight = 2
+++

{{% notice warning %}}
This article refers to the policy on AI used in developing code for use in the RISE codebase. It does not apply to the policy on AI used in reports, presentations and the final documentation in the RISE project that is relevant for grading. To see these guidelines, please refer to the ISIS course. 
{{% /notice %}}

There is a wide variety of tools available now that can be grouped under the name "generative AI" or LLMs that are capable of generating source code. The adoption of these tools is to be expected on a scale that is likley only growing in the future. Various open source projects have adopted a wide range of position (see [here](https://wiki.gentoo.org/wiki/Project:Council/AI_policy), [here](https://lists.debian.org/debian-project/2024/05/msg00003.html), [here](https://asahilinux.org/docs/project/policies/slop/), [here](https://llvm.org/docs/FAQ.html#id4) or [here](https://osralliance.org/wp-content/uploads/2025/05/OSRF-Policy-on-the-Use-of-Generative-Tools-Generative-AI-in-Contributions.pdf)) on the topic of their use in creating code that is to be integrated into the codebase of these projects. For software engineering within the RISE project we adopted the following policy.

**It is forbidden to use generative AI tools to directly generate source code that is comitted to a code base. However, it is allowed to use these tools for guidance, asking questions and help with debugging. It remains the responsibility of the contributor to fully understand the functionality and content of the proposed solution!**

This rule, in itself, is a bad rule, as there is no way of enforcing it. It does not take a genius to use generative AI in a way that it is hardly, if at all recognizable as coming from such a tool. We therefore can, in practice, only ask you to comply with this rule. To explain this decision, see the following rationale. 

- **Quality concerns:** Popular LLms are great at generating code that *plausibly looks* like it works well. For the most part, especially in tasks with overall lower complexity, common models are capable of creating good solutions. However, there is great potential for the introduction of ever so slightly wrong code bits, which are painful to track down later on. 

- **Overestimation:** LLMs also empower people who are not as familiar with the details of a language or framework to tackle tasks that they themself would not be capable of solving. This is great in theory, in practice however it leads to people solving tasks, for which they dont bring the required level of competence, resulting in them being unable to judge the quality and safety of a solution. In addition this is a highly frustrating process for both the person "vibe-coding" the solution as they are not learning much, and the person reviewing the solution as they spend a ,lot of time explaining errors and mistakes that have not been committed by the person receiving the review. 

- **Safety concerns:** While we are not looking to attain a medical device certification anytime soon, we still run the code that is developed here on an exoskeleton with a real person inside of it. Even tough there are reviews in place, and we extensivly test the system before using it with a real pilot, the code written here still, directly or indirectly, controls joints that move real legs. Therefore we require people contributing code here, to **understand their contribution in full detail**.

- **Copyright:** We may want to open source the project at some point. It is still unclear, how AI generated code is treated in this regard and wether or nit it's compatible with the licenses we use. 

So, to summarize: Do not use AI to generate code that you will directly integrate into a pull request. As a rule of thumb, if you typed out all of the code that you submitted, you will be perfectly fine. 

{{% notice tip %}}
If you are interested how the Transformers architecture can be used in robotics, see [this](https://robotics-transformer.github.io/assets/rt1.pdf) interesting publication!
{{% /notice %}}