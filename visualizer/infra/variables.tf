variable "domain" {
  description = "Primary domain for the site"
  type        = string
  default     = "camazotzdiving.com"
}

variable "project_name" {
  description = "Cloudflare Pages project name"
  type        = string
  default     = "camazotz-map"
}

variable "subdomain" {
  description = "Subdomain for the visualizer"
  type        = string
  default     = "map"
}
