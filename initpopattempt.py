# def gen_init_pop(sol_per_pop,num_genes,gene_space,gene_type):
#     # Reaching this block means:
#     # 1) gene_space is nested (gene_space_nested is True).
#     # 2) gene_type is nested (gene_type_single is False).
#     population = np.zeros(shape=(sol_per_pop,num_genes),
#                                     dtype=object)
#     for sol_idx in range(sol_per_pop):
#         for gene_idx in range(num_genes):
#             if type(gene_space[gene_idx]) in [np.ndarray, list, tuple, range]:
#                 # Convert to list because tuple and range do not have copy().
#                 # We copy the gene_space to a temp variable to keep its original value.
#                 # In the next for loop, the gene_space is changed.
#                 # Later, the gene_space is restored to its original value using the temp variable.
#                 temp_gene_space = list(gene_space[gene_idx]).copy()
#                 for idx, val in enumerate(gene_space[gene_idx]):
#                     if val is None:
#                         gene_space[gene_idx][idx] = np.asarray(np.random.uniform(low=-4,
#                                                                                             high=4,
#                                                                                             size=1),
#                                                                         dtype=gene_type[gene_idx][0])[0]
#                 # Check if the gene space has None values. If any, then replace it with randomly generated values according to the 3 attributes init_range_low, init_range_high, and gene_type.
#                 population[sol_idx, gene_idx] = random.choice(gene_space[gene_idx])
#                 population[sol_idx, gene_idx] = gene_type[gene_idx][0](population[sol_idx, gene_idx])
#                 # Restore the gene_space from the temp_gene_space variable.
#                 gene_space[gene_idx] = temp_gene_space.copy()
#             elif type(gene_space[gene_idx]) is dict:
#                 if 'step' in gene_space[gene_idx].keys():
#                     population[sol_idx, gene_idx] = np.asarray(np.random.choice(np.arange(start=gene_space[gene_idx]['low'],
#                                                                                                         stop=gene_space[gene_idx]['high'],
#                                                                                                         step=gene_space[gene_idx]['step']),
#                                                                                             size=1),
#                                                                         dtype=gene_type[gene_idx][0])[0]
#                 else:
#                     population[sol_idx, gene_idx] = np.asarray(np.random.uniform(low=gene_space[gene_idx]['low'],
#                                                                                             high=gene_space[gene_idx]['high'],
#                                                                                             size=1),
#                                                                         dtype=gene_type[gene_idx][0])[0]
#             elif type(gene_space[gene_idx]) in ga.GA.supported_int_float_types:
#                 population[sol_idx, gene_idx] = gene_space[gene_idx]
#             else:
#                 # There is no more options.
#                 pass
#     return population