/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef QC_CAMSS_CPAS_H
#define QC_CAMSS_CPAS_H
struct camss;
struct camss_cpas;
int camss_cpas_init(struct camss *camss);
int camss_cpas_resume(struct camss *camss);
void camss_cpas_suspend(struct camss *camss);
#endif
